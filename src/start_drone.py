#!/usr/bin/env python3
"""
start_drone.py — Sensor Bridge Daemon
=======================================
Reads raw MicoLink frames from two MTF-01P sensors and raw scans from the
RPLidar A1, then packages and forwards them as native MAVLink 2 messages to
the SpeedyBee F405 Mini flight controller over /dev/ttyAMA4.

Thread layout
─────────────
  mtf_down_reader   → reads /dev/ttyAMA0 (floor MTF-01P, 100 Hz raw)
  mtf_up_reader     → reads /dev/ttyAMA2 (ceiling MTF-01P, 100 Hz raw)
  rplidar_reader    → reads /dev/rplidar  (RPLidar A1 scan loop, ~8 Hz)
  mavlink_writer    → pops shared state, encodes & sends MAVLink at fixed rates
  heartbeat_thread  → sends HEARTBEAT to FC every second

All threads are daemon threads — they die automatically when main() exits.
The SIGINT handler cleanly requests shutdown before the process terminates.
"""

import logging
import math
import queue
import signal
import socket
import struct
import sys
import threading
import time
from typing import Optional

import numpy as np
import serial
from pymavlink import mavutil
from rplidar import RPLidar

import config

# ─────────────────────────────────────────────────────────────────────────────
# LOGGING
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(threadName)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("start_drone")


# ─────────────────────────────────────────────────────────────────────────────
# SHARED STATE  — written by sensor threads, read by mavlink_writer
# All fields protected by a single RLock per sensor group.
# ─────────────────────────────────────────────────────────────────────────────

class MtfState:
    """Thread-safe container for one MTF-01P sensor reading."""

    def __init__(self, name: str):
        self.name           = name
        self._lock          = threading.RLock()
        # Range fields
        self.distance_mm: int   = 0        # 0 → unavailable
        self.dis_status: int    = 0
        # Flow fields (downward sensor only; upward sensor ignores these)
        self.flow_vel_x: float  = 0.0      # cm/s @ 1 m
        self.flow_vel_y: float  = 0.0
        self.flow_quality: int  = 0
        # Housekeeping
        self.timestamp_s: float = 0.0      # time.monotonic() of last good frame
        self.frame_count: int   = 0

    def update(self, distance_mm: int, dis_status: int,
               flow_x: float, flow_y: float, quality: int) -> None:
        with self._lock:
            self.distance_mm  = distance_mm
            self.dis_status   = dis_status
            self.flow_vel_x   = flow_x
            self.flow_vel_y   = flow_y
            self.flow_quality = quality
            self.timestamp_s  = time.monotonic()
            self.frame_count += 1

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "distance_mm":  self.distance_mm,
                "dis_status":   self.dis_status,
                "flow_vel_x":   self.flow_vel_x,
                "flow_vel_y":   self.flow_vel_y,
                "flow_quality": self.flow_quality,
                "timestamp_s":  self.timestamp_s,
                "frame_count":  self.frame_count,
            }

    def is_stale(self) -> bool:
        with self._lock:
            return (time.monotonic() - self.timestamp_s) > config.SENSOR_STALE_S


class LidarState:
    """Thread-safe container for the latest RPLidar obstacle sector array."""

    def __init__(self):
        self._lock           = threading.RLock()
        # 72 sectors, each covering 5°; value = closest distance in cm (uint16)
        # Initialised to max range (no obstacle detected)
        self._sectors        = np.full(config.OBSTACLE_SECTORS,
                                       config.OBSTACLE_MAX_CM, dtype=np.uint16)
        self.timestamp_s: float = 0.0
        self.scan_count: int    = 0

    def update(self, sectors: np.ndarray) -> None:
        with self._lock:
            np.copyto(self._sectors, sectors)
            self.timestamp_s = time.monotonic()
            self.scan_count += 1

    def snapshot(self) -> np.ndarray:
        with self._lock:
            return self._sectors.copy()

    def is_stale(self) -> bool:
        with self._lock:
            return (time.monotonic() - self.timestamp_s) > config.SENSOR_STALE_S


# Global shared state objects
mtf_down  = MtfState("MTF-DOWN")
mtf_up    = MtfState("MTF-UP")
lidar     = LidarState()

# Global shutdown event — set to True by SIGINT handler
_shutdown = threading.Event()


# ─────────────────────────────────────────────────────────────────────────────
# MTF-01P MAVLink RELAY  (mav_apm output mode)
# ─────────────────────────────────────────────────────────────────────────────
#
# The MTF-01P sensors are configured in "mav_apm" mode via MicoAssistant.
# In this mode each sensor outputs MAVLink v1 messages on its UART:
#   • OPTICAL_FLOW   (msg 100) — raw pixel flow + quality  (NOT OPTICAL_FLOW_RAD!)
#   • DISTANCE_SENSOR (msg 132) — raw ToF range in cm
#
# NOTE: ArduPilot FLOW_TYPE=5 backend (AP_OpticalFlow_MAV) only accepts
# OPTICAL_FLOW_RAD (msg 106).  The relay thread converts OPTICAL_FLOW →
# OPTICAL_FLOW_RAD on the fly:
#
#   flow_x / flow_y  [dpix = pixels × 10]  →  integrated radians
#   height           from last DISTANCE_SENSOR message
#   integration_time from time_usec delta between consecutive flow messages
#
# This relay thread:
#   1. Opens the sensor UART as a MAVLink connection.
#   2. Receives OPTICAL_FLOW_RAD and DISTANCE_SENSOR packets.
#   3. Updates the shared MtfState so dashboard / status-reporter still work.
#   4. Forwards the packets verbatim to the flight controller (if mav_fc is
#      not None) — the FC cannot tell these came via the Pi rather than from
#      a sensor wired directly to its serial port.
#
# Downward sensor  →  relay_to_fc = True   (flow + rangefinder → EKF)
# Upward sensor    →  relay_to_fc = False  (ceiling distance, monitoring only)


def mtf_mavlink_relay_thread():
    """
    Pure byte-level MAVLink relay from the downward MTF-01P (configured in
    ``mav_apm`` output mode via MicoAssistant) to the flight controller UART.
    """
    print("[MTF Relay] Starting pure MAVLink byte-forwarder...")

    SENSOR_PORT = config.MTF_DOWN_PORT
    FC_PORT = config.FC_SERIAL_PORT
    SENSOR_BAUD = config.MTF_DOWN_BAUD
    FC_BAUD = config.FC_BAUD_RATE
    
    try:
        ser_sensor = serial.Serial(SENSOR_PORT, baudrate=SENSOR_BAUD, timeout=0.01)
        ser_fc = serial.Serial(FC_PORT, baudrate=FC_BAUD, timeout=0.01)
        print(f"[MTF Relay] Connected: Sensor ({SENSOR_PORT}) -> FC ({FC_PORT})")
    except Exception as e:
        print(f"[MTF Relay] Serial init error: {e}")
        return

    ser_sensor.reset_input_buffer()
    ser_fc.reset_output_buffer()

    while True:
        try:
            # Check for incoming raw sensor telemetry
            if ser_sensor.in_waiting > 0:
                raw_bytes = ser_sensor.read(ser_sensor.in_waiting)
                # Stream directly to the Flight Controller without altering the data
                ser_fc.write(raw_bytes)
                
            time.sleep(0.001) # Yield CPU control safely
            
        except Exception as e:
            print(f"[MTF Relay] Error in streaming loop: {e}")
            time.sleep(2)


#
# Two frame types interleaved on the same serial stream:
#
# FRAME TYPE A — ToF RANGE (14 bytes total)
#   [0-1]   0x58 0x3C       Magic header
#   [2]     0x00            System ID
#   [3]     0x01            MSG_ID = Range
#   [4]     0x1F            Fixed protocol byte
#   [5]     0x05            Payload length = 5 bytes
#   [6]     0x00            Fixed
#   [7]     uint8  quality  Signal confidence (255=best, <50=unreliable)
#   [8-11]  uint32 range_mm Little-endian, millimetres
#   [12]    uint8  checksum
#   [13]    0x24   '$'      ETX delimiter
#
# FRAME TYPE B — Optical Flow (18 bytes total)
#   [0-1]   0x58 0x3C       Magic header
#   [2]     0x00            System ID
#   [3]     0x02            MSG_ID = Flow
#   [4]     0x1F            Fixed protocol byte
#   [5]     0x09            Payload length = 9 bytes
#   [6]     0x00            Fixed
#   [7]     uint8  SEQ      Rolling sequence counter 0-255
#   [8-11]  int32  flow_x   Little-endian, mm/s (multiply by 0.001 for m/s)
#   [12-15] int32  flow_y   Little-endian, mm/s
#   [16]    uint8  checksum
#   [17]    0x24   '$'      ETX delimiter
#
# Sync strategy: search for 0x58 0x3C two-byte magic, branch on MSG_ID.

# Protocol constants
_MTF_MAGIC   = bytes([0x58, 0x3C])
_MTF_ETX     = 0x24
_MSG_RANGE   = 0x01
_MSG_FLOW    = 0x02
_LEN_RANGE   = 14
_LEN_FLOW    = 18
_MIN_FRAME   = _LEN_RANGE   # smallest possible frame


def _extract_frame(buf: bytearray):
    """
    Look for the next complete MTF-01P frame in `buf`.

    Returns (frame_bytes, consumed_bytes) on success, or (None, 0) to signal
    'wait for more data', or (None, n>0) to signal 'skip n bytes of junk'.
    """
    # Find magic header
    idx = buf.find(_MTF_MAGIC)
    if idx == -1:
        # No magic at all — discard everything except the last byte
        # (that last byte could be the first byte of a split magic)
        return None, max(0, len(buf) - 1)
    if idx > 0:
        return None, idx   # skip junk before the header

    # We have the magic at position 0
    if len(buf) < 4:
        return None, 0     # need more bytes to read MSG_ID

    msg_id = buf[3]
    if msg_id == _MSG_RANGE:
        expected_len = _LEN_RANGE
    elif msg_id == _MSG_FLOW:
        expected_len = _LEN_FLOW
    else:
        # Unknown msg_id — slide past this magic
        return None, 2

    if len(buf) < expected_len:
        return None, 0     # wait for more bytes

    frame = bytes(buf[:expected_len])

    # Verify ETX footer
    if frame[-1] != _MTF_ETX:
        return None, 1     # footer mismatch — slide one byte

    return frame, expected_len


def parse_mtf_stream(port: str, baud: int, state: MtfState,
                     stop_event: threading.Event) -> None:
    """
    Continuously reads the MTF-01P binary stream from `port` and updates `state`.

    The sensor emits two interleaved frame types (Range 0x01 and Flow 0x02).
    Both are parsed here and merged into the single MtfState object so the
    MAVLink writer always has the freshest range + flow values together.
    """
    ser: Optional[serial.Serial] = None

    # Accumulate partial range/flow within one MtfState update cycle
    _pending_dist_mm: int   = 0
    _pending_quality: int   = 0
    _pending_flow_x:  float = 0.0
    _pending_flow_y:  float = 0.0
    _got_range = False
    _got_flow  = False

    while not stop_event.is_set():
        # ── Open serial port ─────────────────────────────────────────────────────
        try:
            ser = serial.Serial(port, baud, timeout=1.0)
            log.info(f"[{state.name}] Opened {port} @ {baud}")
        except serial.SerialException as exc:
            log.warning(f"[{state.name}] Cannot open {port}: {exc} — retrying in 3 s")
            time.sleep(3.0)
            continue

        buf = bytearray()
        _pending_dist_mm = 0
        _pending_quality = 0
        _pending_flow_x  = 0.0
        _pending_flow_y  = 0.0
        _got_range = False
        _got_flow  = False

        try:
            while not stop_event.is_set():
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                buf.extend(chunk)

                # Keep buffer bounded (discard old data if it grows huge)
                if len(buf) > 512:
                    buf = buf[-256:]

                while len(buf) >= _MIN_FRAME:
                    frame, skip = _extract_frame(buf)

                    if frame is None:
                        if skip > 0:
                            del buf[:skip]
                        break   # wait for more bytes

                    # ── Valid frame — decode ─────────────────────────────────
                    msg_id = frame[3]

                    if msg_id == _MSG_RANGE:
                        # [7] = quality, [8:12] = range_mm (uint32 LE)
                        quality  = frame[7]
                        range_mm = struct.unpack_from('<I', frame, 8)[0]
                        _pending_dist_mm = range_mm
                        _pending_quality = quality
                        _got_range = True
                        log.debug(f"[{state.name}] Range: {range_mm} mm  q={quality}")

                    elif msg_id == _MSG_FLOW:
                        # [8:12] = flow_x (int32 LE mm/s), [12:16] = flow_y
                        flow_x_mms = struct.unpack_from('<i', frame, 8)[0]
                        flow_y_mms = struct.unpack_from('<i', frame, 12)[0]
                        # Convert mm/s → cm/s (MAVLink flow uses cm/s @ 1 m)
                        _pending_flow_x = flow_x_mms * 0.1
                        _pending_flow_y = flow_y_mms * 0.1
                        _got_flow = True
                        log.debug(
                            f"[{state.name}] Flow: x={_pending_flow_x:.1f} "
                            f"y={_pending_flow_y:.1f} cm/s"
                        )

                    # Update shared state whenever we have at least a range reading
                    if _got_range:
                        state.update(
                            _pending_dist_mm,
                            1 if _pending_quality >= config.FLOW_QUALITY_MIN else 0,
                            _pending_flow_x,
                            _pending_flow_y,
                            _pending_quality,
                        )
                        if _got_flow:
                            # Full pair received — reset accumulators
                            _got_range = False
                            _got_flow  = False

                    del buf[:skip if skip > 0 else len(frame)]

        except serial.SerialException as exc:
            log.error(f"[{state.name}] Serial error: {exc} — reconnecting in 2 s")
            time.sleep(2.0)
        except Exception as exc:
            log.exception(f"[{state.name}] Unexpected error: {exc}")
            time.sleep(1.0)
        finally:
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────────────────────
# RPLIDAR READER THREAD
# ─────────────────────────────────────────────────────────────────────────────

def rplidar_reader_thread(stop_event: threading.Event) -> None:
    """
    Reads RPLidar A1 scans and bins them into 72 × 5° sectors.

    Each scan is a list of (quality, angle_deg, distance_mm) tuples.
    We convert to cm, clamp to [OBSTACLE_MIN_CM, OBSTACLE_MAX_CM], and keep
    the minimum distance within each 5° sector (worst-case / closest obstacle).
    """
    rplidar_obj: Optional[RPLidar] = None
    ever_scanned = False   # True once the first scan completes successfully

    while not stop_event.is_set():
        try:
            rplidar_obj = RPLidar(config.RPLIDAR_PORT, baudrate=config.RPLIDAR_BAUD)
            rplidar_obj.start_motor()
            time.sleep(1.0)   # allow motor spin-up
            log.info(f"[RPLIDAR] Connected on {config.RPLIDAR_PORT}")

            for scan in rplidar_obj.iter_scans(max_buf_meas=3000):
                if stop_event.is_set():
                    break

                sectors = np.full(config.OBSTACLE_SECTORS,
                                  config.OBSTACLE_MAX_CM, dtype=np.uint16)

                for meas in scan:
                    try:
                        quality, angle_deg, dist_mm = meas
                    except (TypeError, ValueError):
                        continue
                    if quality < 10 or dist_mm == 0:
                        continue
                    dist_cm = max(config.OBSTACLE_MIN_CM,
                                  min(config.OBSTACLE_MAX_CM,
                                      int(dist_mm / 10.0)))
                    sector_idx = (
                        int(angle_deg / config.OBSTACLE_SECTOR_DEG)
                        % config.OBSTACLE_SECTORS
                    )
                    if dist_cm < sectors[sector_idx]:
                        sectors[sector_idx] = dist_cm

                lidar.update(sectors)
                ever_scanned = True

        except Exception as exc:
            if not ever_scanned:
                # First-connect startup glitch from rplidar-roboticia v0.9.5
                # (double start_motor race on first descriptor read). Normal.
                log.debug(f"[RPLIDAR] Startup sync ({exc}) — reconnecting")
                time.sleep(1.0)
            else:
                log.error(f"[RPLIDAR] Error: {exc} — reconnecting in 3 s")
                time.sleep(3.0)
                ever_scanned = False  # reset so next connect failure is also silent
        finally:
            try:
                if rplidar_obj:
                    rplidar_obj.stop()
                    rplidar_obj.stop_motor()
                    rplidar_obj.disconnect()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────────────────────
# MAVLINK WRITER THREAD
# ─────────────────────────────────────────────────────────────────────────────

def _boot_time_ms() -> int:
    """Milliseconds since process start (used as time_boot_ms in MAVLink)."""
    return int((time.monotonic() - _PROCESS_START) * 1000)


_PROCESS_START = time.monotonic()

# Raw MAVLink packet bytes from the FC, consumed by the TCP bridge thread.
# maxsize=1000 ≈ ~1 second of messages at peak rate — drops on overflow (safe).
_fc_raw_q: queue.Queue = queue.Queue(maxsize=1000)


def mavlink_writer_thread(mav: mavutil.mavfile, stop_event: threading.Event) -> None:
    """
    Regulated send loop — OBSTACLE_DISTANCE only.

    OPTICAL_FLOW_RAD and DISTANCE_SENSOR are now handled directly by
    mtf_mavlink_relay_thread (sensor in mav_apm mode → direct MAVLink relay).
    This thread is responsible only for the RPLidar obstacle sectors at
    OBSTACLE_HZ (10 Hz).
    """
    obstacle_period  = 1.0 / config.OBSTACLE_HZ
    next_obs_t       = time.monotonic()

    MAV_DIST_SENSOR_LASER = 1   # MAV_DISTANCE_SENSOR_LASER
    MAV_FRAME_BODY_FRD    = 12  # MAV_FRAME_BODY_FRD

    log.info("[MAV-WRITER] Send loop started (OBSTACLE_DISTANCE only)")

    while not stop_event.is_set():
        now     = time.monotonic()
        sleep_s = next_obs_t - now
        if sleep_s > 0:
            time.sleep(sleep_s)
        now = time.monotonic()

        # ── OBSTACLE_DISTANCE ─────────────────────────────────────────────────
        if now >= next_obs_t:
            next_obs_t += obstacle_period

            sectors = lidar.snapshot()
            if lidar.is_stale():
                # No fresh data — report max range (no obstacle)
                sectors = np.full(config.OBSTACLE_SECTORS,
                                  config.OBSTACLE_MAX_CM, dtype=np.uint16)

            try:
                mav.mav.obstacle_distance_send(
                    time_usec=int(time.monotonic() * 1e6),
                    sensor_type=MAV_DIST_SENSOR_LASER,
                    distances=sectors.tolist(),
                    increment=config.OBSTACLE_SECTOR_DEG,
                    min_distance=config.OBSTACLE_MIN_CM,
                    max_distance=config.OBSTACLE_MAX_CM,
                    increment_f=float(config.OBSTACLE_SECTOR_DEG),
                    angle_offset=0.0,
                    frame=MAV_FRAME_BODY_FRD,
                )
            except Exception as exc:
                log.debug(f"[MAV-WRITER] obstacle_distance_send error: {exc}")

            # ── Shared-file IPC for auto_navigate.py ──────────────────────────
            # PRX1_TYPE is not compiled into this FC build so the FC cannot echo
            # OBSTACLE_DISTANCE back.  Write the 72-element uint16 sector array
            # to /tmp/drone_obstacles.bin so auto_navigate.py reads it directly.
            try:
                sectors.tofile("/tmp/drone_obstacles.bin")
            except Exception as exc:
                log.debug(f"[MAV-WRITER] obstacle file write error: {exc}")


# ─────────────────────────────────────────────────────────────────────────────
# EKF ORIGIN SEEDING THREAD
# ─────────────────────────────────────────────────────────────────────────────

def ekf_origin_thread(mav: mavutil.mavfile, stop_event: threading.Event) -> None:
    """
    Seeds a fake GPS global origin so ArduPilot accepts Guided-mode velocity
    commands without a real GPS fix.

    Sends SET_GPS_GLOBAL_ORIGIN (MAVLink #48) with lat=lon=alt=0.
    Strategy: send aggressively for the first 10 s (every 2 s = 5 sends),
    then back off to every 30 s for periodic EKF-reset recovery.
    SET_HOME_POSITION is sent only ONCE at startup to avoid spamming the FC
    (each send triggers a STATUSTEXT 'Field Elevation Set: 0m' reply).

    Note: recv_match() is NOT used here because mavlink_reader_thread already
    drains all incoming messages.  Instead we rely on the STATUSTEXT message
    'EKF3 IMU0 origin set' (visible in logs) as proof of acceptance.
    """
    FAKE_LAT_DEG_E7 = 0
    FAKE_LON_DEG_E7 = 0
    FAKE_ALT_MM     = 0

    FAST_INTERVAL   = 2.0    # aggressive sends for first 10 s
    FAST_DURATION   = 10.0   # switch to slow after this many seconds
    SLOW_INTERVAL   = 30.0   # periodic refresh (handles EKF resets)

    start_t           = time.monotonic()
    last_origin_t     = 0.0
    home_sent         = False    # SET_HOME_POSITION sent only once
    transition_logged = False    # "fast-seed done" message printed only once

    log.info("[EKF-ORIGIN] Thread started — seeding fake GPS global origin")

    while not stop_event.is_set():
        now     = time.monotonic()
        elapsed = now - start_t
        interval = FAST_INTERVAL if elapsed < FAST_DURATION else SLOW_INTERVAL

        if now - last_origin_t >= interval:
            last_origin_t = now
            try:
                mav.mav.set_gps_global_origin_send(
                    target_system=mav.target_system,
                    latitude=FAKE_LAT_DEG_E7,
                    longitude=FAKE_LON_DEG_E7,
                    altitude=FAKE_ALT_MM,
                    time_usec=int(time.monotonic() * 1e6),
                )
                log.debug(f"[EKF-ORIGIN] SET_GPS_GLOBAL_ORIGIN sent (t+{elapsed:.0f}s)")
            except Exception as exc:
                log.debug(f"[EKF-ORIGIN] origin send error: {exc}")

        if not home_sent:
            try:
                mav.mav.set_home_position_send(
                    target_system=mav.target_system,
                    latitude=FAKE_LAT_DEG_E7,
                    longitude=FAKE_LON_DEG_E7,
                    altitude=FAKE_ALT_MM,
                    x=0.0, y=0.0, z=0.0,
                    q=[1.0, 0.0, 0.0, 0.0],
                    approach_x=0.0,
                    approach_y=0.0,
                    approach_z=-1.0,
                    time_usec=int(time.monotonic() * 1e6),
                )
                home_sent = True
                log.debug("[EKF-ORIGIN] SET_HOME_POSITION sent (once only)")
            except Exception as exc:
                log.debug(f"[EKF-ORIGIN] home send error: {exc}")

        if not transition_logged and elapsed >= FAST_DURATION:
            transition_logged = True
            log.info("[EKF-ORIGIN] ✓ Fast-seed phase done — switching to 30 s refresh. "
                     "Confirm 'EKF3 IMU0 origin set' appeared above.")

        stop_event.wait(timeout=0.5)



# ─────────────────────────────────────────────────────────────────────────────
# MAVLINK TCP BRIDGE THREAD
# ─────────────────────────────────────────────────────────────────────────────
#
# Linux serial ports are exclusive — only one process can open /dev/ttyAMA4.
# This thread exposes a local TCP server on FC_BRIDGE_PORT (5760) so that
# auto_navigate.py (and Mission Planner / QGC on LAN) can connect as secondary
# GCS clients without competing for the serial port.
#
# Two forwarding directions:
#   FC → TCP  : mavlink_reader_thread puts raw packet bytes in _fc_raw_q;
#               bridge drains the queue and sends to all connected clients.
#   TCP → FC  : bridge reads raw bytes from connected clients and writes
#               directly to the serial port (mav.write()).

def mavlink_tcp_bridge_thread(mav: mavutil.mavfile,
                               stop_event: threading.Event) -> None:
    """
    MAVLink TCP bridge: multiplexes the serial FC connection to TCP clients.
    Supports one client at a time (sufficient for auto_navigate.py use case).
    """
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind(("0.0.0.0", config.FC_BRIDGE_PORT))
        srv.listen(1)
        srv.settimeout(1.0)
        log.info(f"[TCP-BRIDGE] Listening on port {config.FC_BRIDGE_PORT}")
    except OSError as exc:
        log.error(f"[TCP-BRIDGE] Cannot bind port {config.FC_BRIDGE_PORT}: {exc}")
        return

    client: Optional[socket.socket] = None

    while not stop_event.is_set():
        # ── Accept new client ──────────────────────────────────────────────────
        if client is None:
            try:
                client, addr = srv.accept()
                client.settimeout(0.05)
                log.info(f"[TCP-BRIDGE] Client connected from {addr[0]}:{addr[1]}")
            except socket.timeout:
                pass
            except Exception as exc:
                log.debug(f"[TCP-BRIDGE] accept error: {exc}")

        if client is None:
            continue

        # ── FC → TCP: drain raw-message queue and forward ──────────────────────
        try:
            while True:
                raw = _fc_raw_q.get_nowait()
                try:
                    client.sendall(raw)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    log.info("[TCP-BRIDGE] Client disconnected")
                    client.close()
                    client = None
                    break
        except queue.Empty:
            pass

        # ── TCP → FC: forward client commands to serial ────────────────────────
        if client is not None:
            try:
                data = client.recv(4096)
                if data:
                    mav.write(data)  # raw MAVLink bytes → FC serial
                else:
                    log.info("[TCP-BRIDGE] Client disconnected (EOF)")
                    client.close()
                    client = None
            except socket.timeout:
                pass
            except (BrokenPipeError, ConnectionResetError, OSError):
                log.info("[TCP-BRIDGE] Client disconnected")
                client.close()
                client = None
            except Exception as exc:
                log.debug(f"[TCP-BRIDGE] recv error: {exc}")

    srv.close()
    if client:
        client.close()


# ─────────────────────────────────────────────────────────────────────────────
# HEARTBEAT THREAD
# ─────────────────────────────────────────────────────────────────────────────

def heartbeat_thread(mav: mavutil.mavfile, stop_event: threading.Event) -> None:
    """
    Sends a MAVLink HEARTBEAT to the FC every second.
    Without this, ArduPilot may flag the companion link as lost.
    """
    MAV_TYPE_GCS           = 6   # MAV_TYPE_GCS
    MAV_AUTOPILOT_INVALID  = 8   # MAV_AUTOPILOT_INVALID (we are not an autopilot)
    MAV_STATE_ACTIVE       = 4   # MAV_STATE_ACTIVE

    log.info("[HEARTBEAT] Thread started")
    while not stop_event.is_set():
        try:
            mav.mav.heartbeat_send(
                type=MAV_TYPE_GCS,
                autopilot=MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=MAV_STATE_ACTIVE,
            )
        except Exception as exc:
            log.debug(f"[HEARTBEAT] Send error: {exc}")
        stop_event.wait(timeout=1.0 / config.HEARTBEAT_HZ)


# ─────────────────────────────────────────────────────────────────────────────
# MAVLINK READER THREAD  (consumes FC telemetry so the read buffer never fills)
# ─────────────────────────────────────────────────────────────────────────────

def mavlink_reader_thread(mav: mavutil.mavfile, stop_event: threading.Event) -> None:
    """
    Drains incoming MAVLink messages from the FC.
    Also logs occasional status text for debugging.
    Exposes the latest FC heartbeat so auto_navigate.py can poll health.
    """
    log.info("[MAV-READER] Thread started")
    while not stop_event.is_set():
        try:
            msg = mav.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            msg_type = msg.get_type()
            if msg_type == "STATUSTEXT":
                sev = getattr(msg, "severity", 0)
                txt = getattr(msg, "text", "").strip()
                # Suppress repetitive housekeeping messages to DEBUG
                _suppress = ("Field Elevation Set",)
                if any(txt.startswith(s) for s in _suppress):
                    log.debug(f"[FC STATUS] {txt}")
                else:
                    lvl = logging.WARNING if sev <= 4 else logging.INFO
                    log.log(lvl, f"[FC STATUS] {txt}")

            # Forward raw packet bytes to the TCP bridge for secondary GCS clients
            try:
                raw = msg.get_msgbuf()
                if raw:
                    _fc_raw_q.put_nowait(bytes(raw))
            except queue.Full:
                pass   # drop oldest if bridge consumer is slow
            except Exception:
                pass
        except Exception as exc:
            log.debug(f"[MAV-READER] Error: {exc}")


# ─────────────────────────────────────────────────────────────────────────────
# STATUS REPORTER  (prints to stdout every 5 s for monitoring)
# ─────────────────────────────────────────────────────────────────────────────

def status_reporter(stop_event: threading.Event) -> None:
    """Prints a periodic sensor health summary to the console."""
    while not stop_event.is_set():
        stop_event.wait(5.0)
        if stop_event.is_set():
            break
        dn  = mtf_down.snapshot()
        up  = mtf_up.snapshot()
        age_dn = time.monotonic() - dn["timestamp_s"] if dn["timestamp_s"] else 999.0
        age_up = time.monotonic() - up["timestamp_s"] if up["timestamp_s"] else 999.0
        log.info(
            f"[STATUS] MTF-DOWN: dist={dn['distance_mm']}mm "
            f"flow=({dn['flow_vel_x']:.1f},{dn['flow_vel_y']:.1f})cm/s "
            f"q={dn['flow_quality']} age={age_dn:.1f}s | "
            f"MTF-UP: dist={up['distance_mm']}mm age={age_up:.1f}s | "
            f"LIDAR scans={lidar.scan_count}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def sensor_export_thread(stop_event: threading.Event) -> None:
    """
    Writes /tmp/drone_sensors.json at 10 Hz so the dashboard bridge can read
    floor/ceiling distances and optical-flow quality without needing to listen
    to the GCS->FC MAVLink direction (not echoed on TCP bridge).
    """
    import json
    EXPORT_PATH = "/tmp/drone_sensors.json"
    while not stop_event.is_set():
        try:
            dn  = mtf_down.snapshot()
            up  = mtf_up.snapshot()
            now = time.time()  # wall clock — comparable across processes
            # Internal age check uses monotonic (timestamp_s is monotonic in MtfState)
            _mono = time.monotonic()
            age_dn = _mono - dn["timestamp_s"] if dn["timestamp_s"] else 999.0
            age_up = _mono - up["timestamp_s"] if up["timestamp_s"] else 999.0

            floor_m = round(dn["distance_mm"] / 1000.0, 3) \
                if (age_dn < config.SENSOR_STALE_S
                    and config.RNGFND_MIN_MM <= dn["distance_mm"] <= config.RNGFND_MAX_MM) \
                else None

            ceil_m = round(up["distance_mm"] / 1000.0, 3) \
                if (age_up < config.SENSOR_STALE_S
                    and config.RNGFND_MIN_MM <= up["distance_mm"] <= config.RNGFND_MAX_MM) \
                else None

            data = {
                "t":            round(time.time(), 2),  # wall clock for cross-process comparison
                "floor_m":      floor_m,
                "ceil_m":       ceil_m,
                "flow_quality": dn["flow_quality"] if age_dn < config.SENSOR_STALE_S else None,
                "flow_vx":      round(dn["flow_vel_x"], 3) if age_dn < config.SENSOR_STALE_S else None,
                "flow_vy":      round(dn["flow_vel_y"], 3) if age_dn < config.SENSOR_STALE_S else None,
                "lidar_scans":  lidar.scan_count,
            }
            with open(EXPORT_PATH, "w") as f:
                json.dump(data, f)
        except Exception as exc:
            log.debug(f"[sensor_export] {exc}")
        stop_event.wait(0.1)  # 10 Hz


def main() -> None:
    # ── Connect to Flight Controller ──────────────────────────────────────────
    log.info(f"Connecting to FC on {config.FC_SERIAL_PORT} @ {config.FC_BAUD_RATE}...")
    try:
        mav = mavutil.mavlink_connection(
            config.FC_SERIAL_PORT,
            baud=config.FC_BAUD_RATE,
            source_system=config.GCS_SYSTEM_ID,
            source_component=config.GCS_COMPONENT_ID,
        )
    except Exception as exc:
        log.critical(f"Cannot open FC serial port: {exc}")
        sys.exit(1)

    log.info("Waiting for FC heartbeat (will keep waiting until FC is ready)...")
    hb = None
    _hb_attempt = 0
    while hb is None:
        hb = mav.wait_heartbeat(timeout=10)
        if hb is None:
            _hb_attempt += 1
            log.info(f"FC not responding yet (attempt {_hb_attempt}) — FC may still be booting, retrying...")
    log.info(f"FC heartbeat received (sys={mav.target_system}, comp={mav.target_component})")

    # ── Set up MAVLink data streams ───────────────────────────────────────────
    # Request EXTENDED_SYS_STATE + SYS_STATUS for EKF health monitoring
    try:
        mav.mav.request_data_stream_send(
            target_system=mav.target_system,
            target_component=mav.target_component,
            req_stream_id=mavutil.mavlink.MAV_DATA_STREAM_ALL,
            req_message_rate=4,
            start_stop=1,
        )
    except Exception as exc:
        log.warning(f"Could not request data streams: {exc}")

    # ── SIGINT Handler ────────────────────────────────────────────────────────
    def _sigint_handler(sig, frame):
        log.info("SIGINT received — shutting down...")
        _shutdown.set()

    signal.signal(signal.SIGINT, _sigint_handler)
    signal.signal(signal.SIGTERM, _sigint_handler)

    # ── Spawn threads ─────────────────────────────────────────────────────────
    threads = []

    def spawn(name: str, target, *args):
        t = threading.Thread(target=target, args=args, name=name, daemon=True)
        t.start()
        threads.append(t)
        return t

    # Downward sensor: pure byte relay
    spawn("mtf_down_relay", mtf_mavlink_relay_thread)

    # Upward sensor: disabled since pure byte relay hardcodes the port and does not update state
    # spawn("mtf_up_relay", mtf_mavlink_relay_thread,
    #       config.MTF_UP_PORT, config.MTF_UP_BAUD, mtf_up, None, _shutdown)

    spawn("rplidar_reader", rplidar_reader_thread, _shutdown)

    spawn("mavlink_writer", mavlink_writer_thread, mav, _shutdown)

    spawn("tcp_bridge",     mavlink_tcp_bridge_thread, mav, _shutdown)

    spawn("ekf_origin",     ekf_origin_thread,    mav, _shutdown)

    spawn("heartbeat",      heartbeat_thread,      mav, _shutdown)

    spawn("mavlink_reader", mavlink_reader_thread, mav, _shutdown)

    spawn("status_reporter", status_reporter, _shutdown)

    spawn("sensor_export",   sensor_export_thread, _shutdown)

    log.info("All threads running.  Press Ctrl+C to stop.")

    # ── Main loop — keep alive until shutdown ─────────────────────────────────
    while not _shutdown.is_set():
        _shutdown.wait(timeout=1.0)

    log.info("Shutdown complete.")


if __name__ == "__main__":
    main()
