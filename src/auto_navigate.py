#!/usr/bin/env python3
"""
auto_navigate.py — Arming & Wall-Following Navigation Controller
=================================================================
Connects to the Flight Controller via MAVLink (same serial port as
start_drone.py, but this script connects in parallel as a second GCS).

Execution sequence
──────────────────
  PHASE 0  Wait for FC heartbeat
  PHASE 1  Wait for EKF to become healthy (optical flow + rangefinder valid)
  PHASE 2  Switch to GUIDED_NOGPS mode
  PHASE 3  Arm the drone
  PHASE 4  Takeoff to TAKEOFF_ALT_M (1.0 m by default)
  PHASE 5  Wall-following traversal loop (right-hand rule)
  PHASE 6  Land + Disarm  (triggered by SIGINT or max_runtime_s)

Wall-following algorithm
────────────────────────
  At each NAV_LOOP_HZ cycle:
  1.  Read RPLidar obstacle sectors from FC (via DISTANCE message echo)
      OR directly from start_drone.py's shared lidar state if running on same
      process.  This script operates standalone, so it reads OBSTACLE_DISTANCE
      messages reflected back from the FC.
  2.  Extract:
        front_dist  = minimum of sectors [346°–360°] + [0°–14°]  (±15° cone)
        right_dist  = minimum of sectors [76°–104°]               (±15° right cone)
  3.  Compute lateral correction:
        lateral_vel = WALL_P_GAIN * (right_dist - WALL_TARGET_DIST_M)
        (positive = drift left, away from wall; negative = drift right, toward wall)
  4.  State machine:
        CRUISE   → forward at WALL_FOLLOW_SPEED_MS, correct lateral drift
        TURNING  → zero forward, yaw left at TURN_RATE_DEG_S until complete
        STOP_SAFE→ obstacle on all three sides (dead-end) — hover + alert

Usage
──────
  # Full autonomous flight:
  python3 auto_navigate.py

  # Test arm only (no takeoff/navigation):
  python3 auto_navigate.py --arm-only

  # Test mode switch only:
  python3 auto_navigate.py --mode-only

  # Limit runtime (auto-land after N seconds):
  python3 auto_navigate.py --max-runtime 120
"""

import argparse
import logging
import math
import signal
import sys
import time
from enum import Enum, auto
from typing import List, Optional

from pymavlink import mavutil

import config

# ─────────────────────────────────────────────────────────────────────────────
# LOGGING
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("auto_navigate")

# ─────────────────────────────────────────────────────────────────────────────
# MAVLINK HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def connect_fc() -> mavutil.mavfile:
    """Connect to FC via the TCP bridge exposed by start_drone.py."""
    bridge_addr = f"tcp:{config.FC_BRIDGE_HOST}:{config.FC_BRIDGE_PORT}"
    log.info(f"Connecting to FC bridge on {bridge_addr}")
    log.info("  (start_drone.py must already be running — it owns /dev/ttyAMA4)")
    mav = mavutil.mavlink_connection(
        bridge_addr,
        source_system=254,   # secondary GCS ID — won't clash with start_drone.py (255)
        source_component=0,
    )
    log.info("Waiting for FC heartbeat via bridge (timeout 15 s)...")
    hb = mav.wait_heartbeat(timeout=15)
    if hb is None:
        log.critical("No heartbeat received — is start_drone.py running?")
        sys.exit(1)
    log.info(f"FC connected via bridge: sys={mav.target_system} comp={mav.target_component}")
    return mav


# ArduCopter mode numbers (hardcoded — avoids mav.mode_mapping() returning
# plane modes when a GCS heartbeat arrives before the FC heartbeat on the bridge)
COPTER_MODES = {
    "STABILIZE":    0,
    "ALT_HOLD":     2,
    "AUTO":         3,
    "GUIDED":       4,
    "LOITER":       5,
    "RTL":          6,
    "CIRCLE":       7,
    "LAND":         9,
    "GUIDED_NOGPS": 20,
}


def set_mode(mav: mavutil.mavfile, mode_name: str, retries: int = 5) -> bool:
    """
    Command the FC to switch flight mode.
    Uses hardcoded ArduCopter mode numbers to avoid pymavlink mode_mapping()
    returning plane modes when a GCS heartbeat arrives first on the TCP bridge.
    Returns True on success, False on timeout.
    """
    if mode_name not in COPTER_MODES:
        log.error(f"Mode '{mode_name}' not in known Copter mode list: {list(COPTER_MODES.keys())}")
        return False

    mode_id = COPTER_MODES[mode_name]
    for attempt in range(1, retries + 1):
        log.info(f"Setting mode {mode_name} (attempt {attempt}/{retries})...")
        # MAV_CMD_DO_SET_MODE — works regardless of pymavlink's cached mode map
        mav.mav.command_long_send(
            target_system=mav.target_system,
            target_component=mav.target_component,
            command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            confirmation=0,
            param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=float(mode_id),
            param3=0, param4=0, param5=0, param6=0, param7=0,
        )
        # Wait for HEARTBEAT confirming custom_mode matches
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg and msg.custom_mode == mode_id:
                log.info(f"Mode confirmed: {mode_name}")
                return True

    return False


def arm(mav: mavutil.mavfile, retries: int = 5) -> bool:
    """
    Send the MAVLink arm command and wait for confirmation.
    Returns True on success.
    """
    # Flush any stale COMMAND_ACK messages (e.g. from a prior mode-change command)
    # so we don't accidentally consume a mode-change ACK as an ARM ACK.
    while mav.recv_match(type="COMMAND_ACK", blocking=False) is not None:
        pass
    # Also drain any queued STATUSTEXT so we only see fresh ones
    while mav.recv_match(type="STATUSTEXT", blocking=False) is not None:
        pass

    for attempt in range(1, retries + 1):
        log.info(f"Sending ARM command (attempt {attempt}/{retries})...")
        mav.mav.command_long_send(
            target_system=mav.target_system,
            target_component=mav.target_component,
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=1,       # 1 = ARM
            param2=0,       # 0 = normal arm (use 21196 to force past checks)
            param3=0, param4=0, param5=0, param6=0, param7=0,
        )
        # Collect ACKs and STATUSTEXT for up to 3 s
        # STATUSTEXT tells us the exact human-readable rejection reason
        deadline = time.monotonic() + 3.0
        armed = False
        while time.monotonic() < deadline:
            msg = mav.recv_match(type=["COMMAND_ACK", "STATUSTEXT"],
                                 blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                text = msg.text.rstrip("\x00").strip()
                log.warning(f"FC says: {text}")
                continue
            # COMMAND_ACK
            cmd = getattr(msg, "command", None)
            result = getattr(msg, "result", -1)
            if cmd != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                log.debug(f"Ignoring ACK for cmd={cmd} (not ARM)")
                continue
            if result == 0:
                log.info("Armed successfully ✓")
                return True
            else:
                log.warning(f"ARM ACK result={result} — FC refused arm")
                armed = False
                break
        time.sleep(1.0)
    log.error("Failed to arm after all retries")
    return False


def disarm(mav: mavutil.mavfile) -> None:
    """Send the disarm command (best-effort)."""
    log.info("Disarming...")
    mav.mav.command_long_send(
        target_system=mav.target_system,
        target_component=mav.target_component,
        command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation=0,
        param1=0,       # 0 = DISARM
        param2=21196,   # Magic number to force-disarm in-flight if needed
        param3=0, param4=0, param5=0, param6=0, param7=0,
    )


def land(mav: mavutil.mavfile) -> None:
    """Command the FC to land at current position."""
    log.info("Commanding LAND mode...")
    set_mode(mav, "LAND")


def get_current_altitude_m(mav: mavutil.mavfile, timeout: float = 2.0) -> float:
    """
    Returns current altitude in metres from GLOBAL_POSITION_INT or
    VFR_HUD messages.  Falls back to -1 on timeout.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(type=["GLOBAL_POSITION_INT", "VFR_HUD"],
                              blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "VFR_HUD":
            return float(msg.alt)
        if msg.get_type() == "GLOBAL_POSITION_INT":
            return float(msg.relative_alt) / 1000.0
    return -1.0


def send_velocity_body(mav: mavutil.mavfile,
                        vx_ms: float, vy_ms: float, vz_ms: float,
                        yaw_rate_rads: float = 0.0) -> None:
    """
    Send SET_POSITION_TARGET_LOCAL_NED in BODY_NED frame with velocity control.

    vx_ms  — Forward/backward speed (m/s). Positive = forward.
    vy_ms  — Right/left speed (m/s).       Positive = right.
    vz_ms  — Up/down speed (m/s).          Positive = DOWN (NED convention!).
    yaw_rate_rads — Yaw rate (rad/s).      Positive = clockwise/right turn.

    type_mask: ignore position & acceleration, use velocity + yaw-rate.
    Bit 0 (x pos), 1 (y pos), 2 (z pos), 3 (x vel), 4 (y vel), 5 (z vel),
         6 (ax), 7 (ay), 8 (az), 10 (force), 11 (yaw), 12 (yaw_rate)
    Keep bits 3,4,5,12 = 0  →  use vx, vy, vz, yaw_rate
    Mask = 0b1111111111000111 = 0xFFC7
    """
    TYPE_MASK = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    )

    mav.mav.set_position_target_local_ned_send(
        time_boot_ms=int((time.monotonic()) * 1000) & 0xFFFFFFFF,
        target_system=mav.target_system,
        target_component=mav.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask=TYPE_MASK,
        x=0, y=0, z=0,
        vx=vx_ms,
        vy=vy_ms,
        vz=vz_ms,
        afx=0, afy=0, afz=0,
        yaw=0,
        yaw_rate=yaw_rate_rads,
    )


def hover(mav: mavutil.mavfile) -> None:
    """Command zero velocity in all axes (hold position)."""
    send_velocity_body(mav, 0.0, 0.0, 0.0)


# ─────────────────────────────────────────────────────────────────────────────
# EKF HEALTH CHECK
# ─────────────────────────────────────────────────────────────────────────────

# EKF status flags (EKF_STATUS_REPORT flags bitmask)
EKF_FLAG_ATTITUDE      = 0x001
EKF_FLAG_VELOCITY_HORIZ= 0x002
EKF_FLAG_VELOCITY_VERT = 0x004
EKF_FLAG_POS_HORIZ_REL = 0x008
EKF_FLAG_POS_HORIZ_ABS = 0x010
EKF_FLAG_POS_VERT_ABS  = 0x020
EKF_FLAG_POS_VERT_AGL  = 0x040
EKF_FLAG_CONST_POS_MODE= 0x080
EKF_FLAG_PRED_POS_HORIZ_REL = 0x100
EKF_FLAG_PRED_POS_HORIZ_ABS = 0x200

# Minimum flags required before arming in optical-flow-only mode
EKF_REQUIRED_FLAGS = (
    EKF_FLAG_ATTITUDE        |   # attitude solution healthy
    EKF_FLAG_VELOCITY_HORIZ  |   # horizontal velocity from optical flow healthy
    EKF_FLAG_VELOCITY_VERT   |   # vertical velocity healthy
    EKF_FLAG_POS_VERT_AGL        # altitude-above-ground from rangefinder healthy
)


def wait_for_ekf_healthy(mav: mavutil.mavfile, soak_s: float = config.EKF_SOAK_S) -> bool:
    """
    Poll EKF_STATUS_REPORT until the required flags are set for `soak_s` seconds
    continuously.  Returns True when the EKF is confirmed healthy.
    """
    log.info(f"Waiting for EKF health (need {soak_s:.0f} s continuous)...")
    healthy_since: Optional[float] = None

    deadline = time.monotonic() + 120.0   # absolute max wait: 2 minutes

    while time.monotonic() < deadline:
        msg = mav.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=1.0)
        if msg is None:
            healthy_since = None
            log.debug("No EKF_STATUS_REPORT — waiting...")
            continue

        flags = msg.flags
        if (flags & EKF_REQUIRED_FLAGS) == EKF_REQUIRED_FLAGS:
            if healthy_since is None:
                healthy_since = time.monotonic()
                log.info(f"EKF flags OK (0x{flags:04X}) — soak timer started")
            elapsed = time.monotonic() - healthy_since
            if elapsed >= soak_s:
                log.info(f"EKF healthy for {elapsed:.1f} s ✓")
                return True
        else:
            if healthy_since is not None:
                log.warning(f"EKF flags dropped (0x{flags:04X}) — resetting soak timer")
            healthy_since = None

    log.error("EKF did not reach healthy state within timeout")
    return False


# ─────────────────────────────────────────────────────────────────────────────
# LIDAR SECTOR EXTRACTION  (from OBSTACLE_DISTANCE echoed back by FC)
# ─────────────────────────────────────────────────────────────────────────────

class ObstacleSectors:
    """
    Reads the latest RPLidar obstacle sector array written by start_drone.py
    to /tmp/drone_obstacles.bin (a 72-element uint16 binary file).

    This avoids depending on the FC to echo OBSTACLE_DISTANCE messages back,
    which it cannot do since PRX1_TYPE is not compiled into this build.
    """
    SHARED_FILE = "/tmp/drone_obstacles.bin"
    EXPECTED_ELEMENTS = config.OBSTACLE_SECTORS

    def __init__(self):
        self._distances: List[int] = [config.OBSTACLE_MAX_CM] * config.OBSTACLE_SECTORS
        self._updated = False

    def refresh(self) -> None:
        """Read the latest sector data from the shared file (non-blocking)."""
        try:
            import numpy as np
            arr = np.fromfile(self.SHARED_FILE, dtype=np.uint16)
            if len(arr) == self.EXPECTED_ELEMENTS:
                self._distances = arr.tolist()
                self._updated = True
        except FileNotFoundError:
            pass   # start_drone.py hasn't written yet
        except Exception as exc:
            log.debug(f"[OBS] shared-file read error: {exc}")

    def sector_cm(self, angle_deg: float) -> int:
        """Return the obstacle distance (cm) in the given direction."""
        idx = int(round(angle_deg / config.OBSTACLE_SECTOR_DEG)) % config.OBSTACLE_SECTORS
        return self._distances[idx]

    def min_in_cone_cm(self, centre_deg: float, half_width_deg: float = 15.0) -> int:
        """Return the minimum distance in a cone centred at `centre_deg`."""
        n_sectors = int(half_width_deg / config.OBSTACLE_SECTOR_DEG)
        min_d = config.OBSTACLE_MAX_CM
        for delta in range(-n_sectors, n_sectors + 1):
            d = self.sector_cm(centre_deg + delta * config.OBSTACLE_SECTOR_DEG)
            if d < min_d:
                min_d = d
        return min_d

    @property
    def has_data(self) -> bool:
        return self._updated


# ─────────────────────────────────────────────────────────────────────────────
# TAKEOFF
# ─────────────────────────────────────────────────────────────────────────────

def takeoff(mav: mavutil.mavfile, target_alt_m: float = config.TAKEOFF_ALT_M) -> bool:
    """
    Command a GUIDED takeoff to `target_alt_m` metres.
    Monitors RANGEFINDER altitude and returns when within TAKEOFF_TOLERANCE_M.
    """
    log.info(f"Takeoff to {target_alt_m:.1f} m...")

    # ArduPilot GUIDED takeoff via MAV_CMD_NAV_TAKEOFF
    mav.mav.command_long_send(
        target_system=mav.target_system,
        target_component=mav.target_component,
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        confirmation=0,
        param1=0,   # min pitch
        param2=0, param3=0, param4=0,   # unused for copter
        param5=0, param6=0,
        param7=target_alt_m,            # target altitude (m)
    )

    log.info("Climbing — waiting for target altitude...")
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        alt = get_current_altitude_m(mav, timeout=1.0)
        if alt >= 0:
            log.info(f"  Altitude: {alt:.2f} m / {target_alt_m:.2f} m")
            if abs(alt - target_alt_m) <= config.TAKEOFF_TOLERANCE_M:
                log.info("Target altitude reached ✓")
                return True
        time.sleep(0.5)

    log.warning("Takeoff timeout — proceeding at current altitude")
    return False


# ─────────────────────────────────────────────────────────────────────────────
# WALL-FOLLOWING STATE MACHINE
# ─────────────────────────────────────────────────────────────────────────────

class NavState(Enum):
    CRUISE   = auto()   # Moving forward, maintaining right-wall distance
    TURNING  = auto()   # Yawing left 90° (right-hand rule wall follow)
    STOP_SAFE= auto()   # Dead-end / obstacle all around — hover in place


def wall_follow_loop(mav: mavutil.mavfile,
                     obs: ObstacleSectors,
                     max_runtime_s: float,
                     shutdown_event) -> None:
    """
    Main wall-following navigation loop.

    Right-hand rule:
    - Normally, cruise forward while keeping a wall ~60 cm to the right.
    - If something appears < FRONT_BRAKE_DIST_M ahead, stop forward motion and
      yaw left by 90° (turn away from the obstacle).
    - If both front AND left are blocked, hover and log a dead-end warning.
    """
    TURN_RATE_RADS = math.radians(config.TURN_RATE_DEG_S)

    state = NavState.CRUISE
    turn_start_t = 0.0
    last_cmd_t   = 0.0
    start_t      = time.monotonic()

    log.info("Wall-following loop started. Ctrl+C to stop.")

    while not shutdown_event.is_set():
        loop_start = time.monotonic()

        # ── Check max runtime ─────────────────────────────────────────────────
        if (time.monotonic() - start_t) >= max_runtime_s:
            log.info(f"Max runtime ({max_runtime_s:.0f} s) reached — landing")
            break

        # ── Read obstacle sectors from shared file (written by start_drone.py) ──────
        obs.refresh()

        # Drain STATUSTEXT and other FC messages (non-blocking)
        while True:
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == "STATUSTEXT":
                sev = getattr(msg, "severity", 0)
                txt = getattr(msg, "text", "")
                if sev <= 4:
                    log.warning(f"[FC] {txt}")

        if not obs.has_data:
            log.debug("No obstacle data yet — start_drone.py not running? — hovering")
            hover(mav)
            time.sleep(config.NAV_LOOP_S)
            continue

        # ── Read obstacle distances ───────────────────────────────────────────
        front_cm = obs.min_in_cone_cm(0.0,   half_width_deg=15.0)
        right_cm = obs.min_in_cone_cm(90.0,  half_width_deg=15.0)
        left_cm  = obs.min_in_cone_cm(270.0, half_width_deg=15.0)

        front_m = front_cm / 100.0
        right_m = right_cm / 100.0

        log.info(
            f"[NAV {state.name}] front={front_cm}cm right={right_cm}cm left={left_cm}cm"
        )

        # ── State machine ─────────────────────────────────────────────────────

        if state == NavState.CRUISE:
            if front_m < config.FRONT_BRAKE_DIST_M:
                # Obstacle ahead — check if also blocked on left
                if left_cm < int(config.FRONT_BRAKE_DIST_M * 100):
                    log.warning("Blocked front AND left — entering STOP_SAFE")
                    state = NavState.STOP_SAFE
                else:
                    log.info("Obstacle ahead — turning left 90°")
                    state     = NavState.TURNING
                    turn_start_t = time.monotonic()
                    hover(mav)
            else:
                # Cruise forward with right-wall correction
                lateral_err = right_m - config.WALL_TARGET_DIST_M
                vy = config.WALL_P_GAIN * lateral_err   # positive = drift right (toward wall)
                vy = max(-0.3, min(0.3, vy))             # clamp lateral correction

                vx = config.WALL_FOLLOW_SPEED_MS

                # Ceiling guard — do not pitch up if ceiling is too close
                # (vz is zero in cruise, so this is a precaution for altitude hold)

                log.debug(f"CRUISE: vx={vx:.2f} vy={vy:.2f}")
                send_velocity_body(mav, vx_ms=vx, vy_ms=vy, vz_ms=0.0)
                last_cmd_t = time.monotonic()

        elif state == NavState.TURNING:
            elapsed_turn = time.monotonic() - turn_start_t
            if elapsed_turn >= config.TURN_90_DEG_S:
                log.info("90° turn complete — resuming CRUISE")
                state = NavState.CRUISE
                hover(mav)
            else:
                # Yaw left (negative yaw_rate in NED = counter-clockwise = left)
                send_velocity_body(mav, vx_ms=0.0, vy_ms=0.0, vz_ms=0.0,
                                   yaw_rate_rads=-TURN_RATE_RADS)
                last_cmd_t = time.monotonic()

        elif state == NavState.STOP_SAFE:
            # Re-evaluate every loop — exit if front clears
            if front_m >= config.FRONT_BRAKE_DIST_M:
                log.info("Front cleared — resuming CRUISE")
                state = NavState.CRUISE
            else:
                hover(mav)
                last_cmd_t = time.monotonic()

        # ── Velocity command keepalive ─────────────────────────────────────────
        # ArduPilot drops velocity control after ~3 s without a new command.
        # If we haven't sent anything for a while (e.g. state is STOP_SAFE and
        # front is blocked), re-send hover to prevent unexpected drift.
        if (time.monotonic() - last_cmd_t) > config.VEL_CMD_REPEAT_S:
            hover(mav)
            last_cmd_t = time.monotonic()

        # ── Loop timing ───────────────────────────────────────────────────────
        elapsed = time.monotonic() - loop_start
        sleep_s = config.NAV_LOOP_S - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)


# ─────────────────────────────────────────────────────────────────────────────
# ARGUMENT PARSING
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Autonomous wall-following drone controller (no ROS)"
    )
    p.add_argument(
        "--arm-only", action="store_true",
        help="Arm the drone and hover — do NOT takeoff or navigate",
    )
    p.add_argument(
        "--mode-only", action="store_true",
        help="Switch to GUIDED mode only — do NOT arm",
    )
    p.add_argument(
        "--max-runtime", type=float, default=300.0, metavar="SECONDS",
        help="Auto-land after this many seconds of navigation (default: 300)",
    )
    p.add_argument(
        "--skip-ekf-check", action="store_true",
        help="Skip EKF health soak (useful for bench testing without sensors)",
    )
    return p.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

_shutdown = False

def _sigint_handler(sig, frame):
    global _shutdown
    log.info("SIGINT received — initiating safe landing")
    _shutdown = True

signal.signal(signal.SIGINT, _sigint_handler)
signal.signal(signal.SIGTERM, _sigint_handler)


def main():
    global _shutdown
    args = parse_args()

    # ── Connect ───────────────────────────────────────────────────────────────
    mav = connect_fc()

    # ── Phase 1: EKF health ───────────────────────────────────────────────────
    if not args.skip_ekf_check and not args.mode_only:
        ok = wait_for_ekf_healthy(mav, soak_s=config.EKF_SOAK_S)
        if not ok:
            log.critical("EKF never healthy — aborting")
            sys.exit(1)
    else:
        log.info("EKF check skipped")

    if _shutdown:
        sys.exit(0)

    # ── Phase 2: GUIDED mode ──────────────────────────────────────────────────
    if not set_mode(mav, "GUIDED_NOGPS"):
        log.critical("Cannot switch to GUIDED_NOGPS mode — aborting")
        sys.exit(1)

    if args.mode_only:
        log.info("--mode-only flag set — done.")
        sys.exit(0)

    if _shutdown:
        sys.exit(0)

    # ── Phase 3: Arm ──────────────────────────────────────────────────────────
    if not arm(mav):
        log.critical("Arming failed — aborting")
        sys.exit(1)

    if args.arm_only:
        log.info("--arm-only flag set — hovering. Ctrl+C to land.")
        try:
            while not _shutdown:
                hover(mav)
                time.sleep(0.5)
        finally:
            land(mav)
            time.sleep(5.0)
            disarm(mav)
        sys.exit(0)

    if _shutdown:
        land(mav)
        disarm(mav)
        sys.exit(0)

    # ── Phase 4: Takeoff ──────────────────────────────────────────────────────
    takeoff(mav, target_alt_m=config.TAKEOFF_ALT_M)

    if _shutdown:
        land(mav)
        time.sleep(6.0)
        disarm(mav)
        sys.exit(0)

    # ── Phase 5: Wall-following ───────────────────────────────────────────────
    obs = ObstacleSectors()

    # Small class to wrap _shutdown as an event-like object for the loop
    class ShutdownEvent:
        def is_set(self):
            return _shutdown

    try:
        wall_follow_loop(mav, obs, max_runtime_s=args.max_runtime,
                         shutdown_event=ShutdownEvent())
    except Exception as exc:
        log.exception(f"Navigation loop error: {exc}")
    finally:
        # ── Phase 6: Land + Disarm ────────────────────────────────────────────
        log.info("Navigation ended — landing")
        hover(mav)
        time.sleep(0.5)
        land(mav)
        log.info("Landing... waiting 8 s for touchdown")
        time.sleep(8.0)
        disarm(mav)
        log.info("Disarmed. Bye.")


if __name__ == "__main__":
    main()
