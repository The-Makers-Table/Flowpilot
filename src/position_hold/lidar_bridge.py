#!/usr/bin/env python3
"""
lidar_bridge.py — RPLidar A1M8 → MAVLink OBSTACLE_DISTANCE Bridge
==================================================================
Runs continuously on Raspberry Pi (as a systemd service).

Reads RPLidar A1M8 scan data over USB and sends it to the Cube Orange
flight controller as MAVLink OBSTACLE_DISTANCE messages.
ArduPilot uses this for proximity-based obstacle avoidance in Loiter mode.

Hardware:
  - RPLidar A1M8 → RPi USB  → /dev/ttyUSB0
  - RPi GPIO14 (TXD, Pin 8) → Cube Orange GPS1/SERIAL3 Pin 3 (RX)
  - RPi GPIO15 (RXD, Pin10) → Cube Orange GPS1/SERIAL3 Pin 2 (TX)
  - RPi GND        (Pin 6)  → Cube Orange GPS1/SERIAL3 Pin 8 (GND)

ArduPilot parameters required (set in Mission Planner):
  SERIAL3_PROTOCOL = 2     (MAVLink2)
  SERIAL3_BAUD     = 921600
  BRD_SER3_RTSCTS  = 0     (disable flow control)
  PRX1_TYPE        = 2     (MAVLink proximity)
  PRX1_ORIENT      = 0     (horizontal)
  AVOID_ENABLE     = 3     (proximity + fence)
  AVOID_BEHAVE     = 1     (Stop mode)
  AVOID_MARGIN     = 0.7   (stop 0.7 m from wall)
  AVOID_BACKUP_SPD = 0.3
"""

import os
import time
import sys
import logging
from rplidar import RPLidar
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION — override via environment variables or edit defaults below
# ─────────────────────────────────────────────────────────────────────────────

LIDAR_PORT = os.environ.get('RPLIDAR_PORT', '/dev/ttyUSB0')   # RPLidar A1M8 USB port
FC_PORT    = os.environ.get('FC_SERIAL_PORT', '/dev/ttyAMA0') # RPi GPIO UART (after disabling Bluetooth)
FC_BAUD    = int(os.environ.get('FC_BAUD_RATE', '921600'))    # Must match SERIAL3/4_BAUD on Cube Orange

# RPLidar A1M8 sensor limits
MIN_DIST_CM = 15                  # Minimum valid distance (15 cm)
MAX_DIST_CM = 1200                # Maximum valid distance (12 m)

# OBSTACLE_DISTANCE message config
NUM_SECTORS   = 72                # 72 sectors × 5°/sector = 360°
SECTOR_DEG    = 360.0 / NUM_SECTORS  # 5.0 degrees per sector

# Lidar mounting angle offset (degrees):
# Set to the RPLidar angle (0–360) that points to the FRONT of the drone.
# Example: if the lidar cable faces backward, front = 0°, so offset = 0.
# Rotate by 90 if cable faces left, 180 if cable faces front, etc.
LIDAR_FORWARD_ANGLE = 0          # CONFIGURE ME: angle on lidar = drone forward

# How often to send OBSTACLE_DISTANCE messages (Hz)
SEND_RATE_HZ = 10

# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)


def scan_to_distances(scan):
    """
    Convert one RPLidar scan to a 72-element distance array (cm).

    RPLidar gives angles counter-clockwise (CCW) from some reference.
    MAVLink OBSTACLE_DISTANCE expects clockwise (CW) from drone forward.

    Unknown / out-of-range sectors are filled with 65535 (ArduPilot ignores them).
    """
    distances = [65535] * NUM_SECTORS  # Start as all-unknown

    for _, lidar_angle, dist_mm in scan:
        if dist_mm == 0:
            continue

        dist_cm = int(dist_mm / 10.0)
        if dist_cm < MIN_DIST_CM or dist_cm > MAX_DIST_CM:
            continue

        # 1. Make angle relative to drone forward direction
        rel_angle = (lidar_angle - LIDAR_FORWARD_ANGLE) % 360.0

        # 2. RPLidar rotates CCW, but MAVLink expects CW angles.
        #    Convert: CW_angle = 360 - CCW_angle
        mav_angle = (360.0 - rel_angle) % 360.0

        # 3. Map to sector index (each sector = 5°)
        sector = int(mav_angle / SECTOR_DEG) % NUM_SECTORS

        # Keep the closest reading per sector
        if distances[sector] == 65535 or dist_cm < distances[sector]:
            distances[sector] = dist_cm

    return distances


def send_obstacle_distance(mav, distances):
    """
    Send MAVLink OBSTACLE_DISTANCE message (#330) to ArduPilot.
    ArduPilot uses this to populate the PRX (proximity) system and
    trigger AVOID_BEHAVE=1 (Stop) when obstacles are within AVOID_MARGIN.
    """
    mav.mav.obstacle_distance_send(
        int(time.time() * 1e6),   # time_usec: UNIX timestamp in microseconds
        0,                         # sensor_type: 0 = LASER
        distances,                 # distances[72]: cm, 65535 = unknown
        0,                         # increment (legacy, use increment_f instead)
        MIN_DIST_CM,               # min_distance: cm
        MAX_DIST_CM,               # max_distance: cm
        SECTOR_DEG,                # increment_f: degrees per sector (float)
        0.0,                       # angle_offset: 0 = sector[0] points forward
        12                         # frame: MAV_FRAME_BODY_FRD (12)
    )


def connect_fc(port, baud, retries=10):
    """Connect to flight controller with retry logic."""
    for attempt in range(1, retries + 1):
        try:
            log.info(f"Connecting to FC on {port} at {baud} baud (attempt {attempt}/{retries})...")
            mav = mavutil.mavlink_connection(port, baud=baud)
            mav.wait_heartbeat(timeout=5)
            log.info(f"FC connected. System={mav.target_system}, Component={mav.target_component}")
            return mav
        except Exception as e:
            log.warning(f"Connection failed: {e}")
            time.sleep(3)
    log.error("Could not connect to FC after all retries. Check wiring and parameters.")
    sys.exit(1)


def main():
    log.info("=== RPLidar A1M8 → MAVLink Bridge starting ===")

    # Connect to Cube Orange
    mav = connect_fc(FC_PORT, FC_BAUD)

    # Connect to RPLidar
    log.info(f"Opening RPLidar on {LIDAR_PORT}...")
    lidar = RPLidar(LIDAR_PORT, baudrate=115200)
    lidar.start_motor()
    time.sleep(1.0)  # Let motor spin up

    info = lidar.get_info()
    health = lidar.get_health()
    log.info(f"Lidar info: {info}")
    log.info(f"Lidar health: {health}")

    if health[0] == 'Warning':
        log.warning("Lidar health warning — check sensor!")
    elif health[0] == 'Error':
        log.error("Lidar health error — aborting!")
        lidar.stop()
        lidar.disconnect()
        sys.exit(1)

    log.info("Streaming OBSTACLE_DISTANCE messages to ArduPilot...")
    log.info(f"Send rate: {SEND_RATE_HZ} Hz | Sectors: {NUM_SECTORS} × {SECTOR_DEG}°")

    send_interval = 1.0 / SEND_RATE_HZ
    last_send_time = 0
    scan_count = 0

    try:
        for scan in lidar.iter_scans(max_buf_meas=500):
            now = time.time()

            # Rate-limit how often we send MAVLink messages
            if now - last_send_time < send_interval:
                continue

            distances = scan_to_distances(scan)
            send_obstacle_distance(mav, distances)
            last_send_time = now
            scan_count += 1

            # Log a summary every 50 sends (~5 seconds)
            if scan_count % 50 == 0:
                valid = sum(1 for d in distances if d != 65535)
                min_d = min((d for d in distances if d != 65535), default=0)
                log.info(f"Scan #{scan_count} | Valid sectors: {valid}/72 | Closest: {min_d} cm")

    except KeyboardInterrupt:
        log.info("Interrupted by user.")
    except Exception as e:
        log.error(f"Unexpected error: {e}", exc_info=True)
    finally:
        log.info("Shutting down lidar...")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        log.info("Done.")


if __name__ == '__main__':
    main()
