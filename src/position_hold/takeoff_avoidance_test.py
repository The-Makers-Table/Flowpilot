#!/usr/bin/env python3
"""
takeoff_avoidance_test.py — Obstacle Avoidance Flight Test Script
=================================================================
Does the same takeoff as takeoff.py, BUT configures the avoidance
parameters specifically for safe indoor testing:

  - AVOID_BEHAVE  = 0  (SLIDE mode instead of STOP)
    Drone slides along walls rather than freezing in place.
    You can fly freely — it just won't let you go THROUGH walls.

  - AVOID_MARGIN  = 0.5m  (reduced from 0.7m for indoor testing)
    Less restrictive so the drone can move in small rooms.
    Change back to 0.7m for real flights.

  - AVOID_BACKUP_SPD = 0.5 m/s  (increased from 0.3)
    Drone backs away more noticeably when something is too close.

After takeoff and LOITER, this script stays running and prints
a live proximity monitor so you can see exactly what the lidar
is detecting and when avoidance is triggering.

Press Ctrl+C to stop the monitor (drone keeps flying in LOITER).
Original avoidance parameters are restored when you Ctrl+C.

Usage:
  python3 takeoff_avoidance_test.py
"""

import os
import time
import sys
import logging
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

FC_PORT    = os.environ.get('FC_SERIAL_PORT', '/dev/ttyAMA0') # RPi GPIO UART
FC_BAUD    = int(os.environ.get('FC_BAUD_RATE', '921600'))    # Must match SERIAL3/4_BAUD on Cube Orange

TARGET_ALT = 1.5              # Takeoff altitude in metres
ALT_TOLERANCE = 0.10          # Within this metres = target reached (±10 cm)

# EKF origin — use 0,0 for indoor no-GPS flight.
EKF_ORIGIN_LAT = 0.0
EKF_ORIGIN_LON = 0.0
EKF_ORIGIN_ALT = 0.0

ARM_TIMEOUT  = 15
ALT_TIMEOUT  = 30
EKF_TIMEOUT  = 45

# ── AVOIDANCE TEST PARAMETERS ─────────────────────────────────────────────────
# These replace the production settings DURING the test only.
# Ctrl+C restores the originals automatically.
TEST_AVOID_BEHAVE    = 0      # 0=SLIDE (allows movement along walls, not through)
                               # vs production STOP (1) which freezes in all directions
TEST_AVOID_MARGIN    = 0.5    # metres — less restrictive for small indoor rooms
TEST_AVOID_BACKUP    = 0.5    # m/s — faster pushback so you can clearly see it work

# Production values restored on Ctrl+C
PROD_AVOID_BEHAVE    = 1
PROD_AVOID_MARGIN    = 0.7
PROD_AVOID_BACKUP    = 0.3

# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)


def connect_fc(port, baud):
    """Connect to Cube Orange and return MAVLink connection."""
    log.info(f"Connecting to flight controller on {port} @ {baud}...")
    mav = mavutil.mavlink_connection(port, baud=baud)
    log.info("Waiting for heartbeat...")
    mav.wait_heartbeat(timeout=10)
    log.info(f"Connected! System={mav.target_system}, Component={mav.target_component}")
    
    # Request telemetry streams explicitly (10Hz) to ensure EKF status messages arrive
    log.info("Requesting telemetry streams from FC...")
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        0,   # MAV_DATA_STREAM_ALL
        10,  # 10 Hz
        1    # Start streaming
    )
    return mav


def set_mode(mav, mode_name):
    """Set ArduCopter flight mode by name."""
    mode_id = mav.mode_mapping().get(mode_name)
    if mode_id is None:
        log.error(f"Unknown mode: {mode_name}. Available: {list(mav.mode_mapping().keys())}")
        sys.exit(1)

    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    # Confirm mode change
    deadline = time.time() + 5
    while time.time() < deadline:
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_id:
            log.info(f"Mode set to: {mode_name}")
            return
    log.warning(f"Mode confirmation timed out (may still have worked).")


def set_ekf_origin(mav):
    """
    Send SET_GPS_GLOBAL_ORIGIN to give the EKF a position reference.
    Required for no-GPS (optical flow) flight — without this, ArduPilot
    won't let you arm in GUIDED mode.
    """
    log.info("Setting EKF origin...")
    mav.mav.set_gps_global_origin_send(
        mav.target_system,
        int(EKF_ORIGIN_LAT * 1e7),   # latitude  in 1e-7 degrees
        int(EKF_ORIGIN_LON * 1e7),   # longitude in 1e-7 degrees
        int(EKF_ORIGIN_ALT * 1e3)    # altitude  in mm
    )
    log.info(f"EKF origin set to ({EKF_ORIGIN_LAT}, {EKF_ORIGIN_LON}, {EKF_ORIGIN_ALT}m)")


def wait_for_ekf(mav, timeout=EKF_TIMEOUT):
    """
    Wait until the EKF reports a healthy position estimate.
    Checks EKF_STATUS_REPORT flags — specifically velocity and position variance.
    """
    log.info("Waiting for EKF to initialise...")
    deadline = time.time() + timeout

    while time.time() < deadline:
        msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
        if msg is None:
            continue

        flags = msg.flags
        # EKF_STATUS_REPORT flags (bitmask):
        # bit 0: attitude_estimate_ok
        # bit 1: velocity_horiz_estimate_ok
        # bit 2: velocity_vert_estimate_ok
        # bit 3: pos_horiz_rel_ok    (relative horizontal position — what we need)
        # bit 4: pos_horiz_abs_ok
        # bit 5: pos_vert_abs_ok
        # bit 6: pos_vert_agl_ok    (AGL height, needs rangefinder)
        # bit 7: const_pos_mode_ok  (constant position mode — set when NOT moving)
        # bit 8: pred_pos_horiz_rel_ok
        # bit 9: pred_pos_horiz_abs_ok

        attitude_ok  = bool(flags & (1 << 0))
        vel_horiz_ok = bool(flags & (1 << 1))
        pos_rel_ok   = bool(flags & (1 << 3))
        pos_vert_ok  = bool(flags & (1 << 6))  # AGL from rangefinder

        log.info(
            f"EKF flags=0x{flags:04X} | "
            f"attitude={'OK' if attitude_ok else '--'} "
            f"vel_horiz={'OK' if vel_horiz_ok else '--'} "
            f"pos_rel={'OK' if pos_rel_ok else '--'} "
            f"pos_vert={'OK' if pos_vert_ok else '--'}"
        )

        # Minimum requirement: attitude + velocity estimate ready
        # Full position hold also needs pos_rel_ok (optical flow working)
        if attitude_ok and vel_horiz_ok:
            log.info("EKF initialised — attitude and velocity estimates ready.")
            if not pos_rel_ok:
                log.warning(
                    "EKF position estimate not fully ready. "
                    "Optical flow may not be working yet. "
                    "Proceeding anyway — position hold quality may be reduced."
                )
            return True

        time.sleep(0.5)

    log.error(f"EKF did not initialise within {timeout}s. Check sensor connections.")
    return False


def arm_vehicle(mav, timeout=ARM_TIMEOUT):
    """Arm the vehicle and wait for confirmation."""
    log.info("Arming vehicle...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,      # confirmation
        1,      # param1: 1 = arm, 0 = disarm
        0, 0, 0, 0, 0, 0
    )

    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                log.info("Vehicle armed!")
                return True

        # Check for command acknowledgement
        ack = mav.recv_match(type='COMMAND_ACK', blocking=False)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                result_name = mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
                log.error(f"Arm rejected: {result_name}")
                log.error("Common causes: pre-arm checks failing (check Mission Planner)")
                return False

    log.error(f"Arm timed out after {timeout}s.")
    return False


def takeoff(mav, altitude, timeout=ALT_TIMEOUT):
    """Send NAV_TAKEOFF command and wait until target altitude is reached."""
    log.info(f"Sending takeoff command to {altitude}m...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,           # confirmation
        0,           # param1: minimum pitch (ignored for copter)
        0,           # param2: empty
        0,           # param3: empty
        float('nan'),  # param4: yaw (NaN = current heading)
        0,           # param5: lat (0 = current)
        0,           # param6: lon (0 = current)
        altitude     # param7: altitude in metres
    )

    log.info(f"Climbing to {altitude}m...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg is None:
            continue

        current_alt = msg.relative_alt / 1000.0  # mm → metres
        remaining   = altitude - current_alt
        log.info(f"  Alt: {current_alt:.2f}m / {altitude}m  (remaining: {remaining:.2f}m)")

        if abs(current_alt - altitude) <= ALT_TOLERANCE:
            log.info(f"Target altitude {altitude}m reached!")
            return True

        time.sleep(0.2)

    log.error(f"Takeoff timed out after {timeout}s. Last alt: {current_alt:.2f}m")
    return False


def get_statustext(mav, timeout=0.2):
    """Drain any STATUSTEXT messages and print them (FC pre-arm messages etc)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            log.info(f"FC says: [{msg.severity}] {msg.text}")


def main():
    log.info("=== Autonomous Takeoff Script ===")
    log.info(f"Target altitude: {TARGET_ALT}m")

    # Step 1: Connect to Cube Orange
    mav = connect_fc(FC_PORT, FC_BAUD)
    time.sleep(1)

    # Step 2: Set EKF origin (required for no-GPS arming)
    set_ekf_origin(mav)
    time.sleep(0.5)

    # Step 3: Switch to GUIDED mode
    log.info("Setting GUIDED mode...")
    set_mode(mav, 'GUIDED')
    time.sleep(1)

    # Step 4: Wait for EKF to be ready
    if not wait_for_ekf(mav):
        log.error("Aborting: EKF not ready.")
        sys.exit(1)

    # Step 5: Arm
    if not arm_vehicle(mav):
        log.error("Aborting: Failed to arm.")
        sys.exit(1)

    time.sleep(1)  # Short pause after arming before takeoff

    # Step 6: Take off
    if not takeoff(mav, TARGET_ALT):
        log.error("Takeoff failed or timed out. Disarming for safety...")
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0  # param1=0 → disarm
        )
        sys.exit(1)

    # Step 7: Switch to LOITER (position hold using optical flow)
    log.info("Switching to LOITER mode for position hold...")
    set_mode(mav, 'LOITER')
    time.sleep(1)
    get_statustext(mav, timeout=2)

    # ── Apply test avoidance parameters ───────────────────────────────────────
    log.info("")
    log.info("Applying TEST avoidance parameters...")
    def set_param(name, value):
        mav.mav.param_set_send(
            mav.target_system, mav.target_component,
            name.encode(), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.3)

    set_param('AVOID_BEHAVE',    TEST_AVOID_BEHAVE)
    set_param('AVOID_MARGIN',    TEST_AVOID_MARGIN)
    set_param('AVOID_BACKUP_SPD', TEST_AVOID_BACKUP)

    log.info(f"  AVOID_BEHAVE    = {TEST_AVOID_BEHAVE}  (SLIDE — drone moves along walls, not through them)")
    log.info(f"  AVOID_MARGIN    = {TEST_AVOID_MARGIN}m (stop {TEST_AVOID_MARGIN}m from obstacle)")
    log.info(f"  AVOID_BACKUP_SPD= {TEST_AVOID_BACKUP} m/s (active pushback speed)")
    log.info("")
    log.info("=" * 60)
    log.info("✅ DRONE IN LOITER — Take control with your RC transmitter")
    log.info("   Fly toward a wall — drone will SLIDE along it, not stop dead")
    log.info("   Bring hand near lidar — drone will push away")
    log.info("   Press Ctrl+C to stop monitor and restore production params")
    log.info("=" * 60)

    # ── Live proximity monitor ─────────────────────────────────────────────────
    log.info("")
    log.info("[LIVE PROXIMITY MONITOR]")
    log.info("Format: Front | Right | Back | Left | Closest")
    log.info("")

    try:
        while True:
            # Request fresh data
            mav.mav.request_data_stream_send(
                mav.target_system, mav.target_component,
                0, 5, 1
            )

            # Grab a DISTANCE_SENSOR message (sent by proximity system)
            msg = mav.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=1)

            # Also check for proximity data via named distances
            prx = mav.recv_match(type='PROXIMITY', blocking=False)

            # Read EKF / heartbeat for mode
            hb = mav.recv_match(type='HEARTBEAT', blocking=False)
            mode = ""
            if hb:
                mode_map = {v: k for k, v in mav.mode_mapping().items()}
                mode = mode_map.get(hb.custom_mode, "?")

            if prx:
                # PROXIMITY message has distance[8] in metres for 8 sectors
                # Sectors: 0=front, 2=right, 4=back, 6=left
                d = prx.distance
                front = f"{d[0]:.1f}m" if d[0] > 0 else "--"
                right = f"{d[2]:.1f}m" if d[2] > 0 else "--"
                back  = f"{d[4]:.1f}m" if d[4] > 0 else "--"
                left  = f"{d[6]:.1f}m" if d[6] > 0 else "--"
                valid = [x for x in d if x > 0]
                closest = f"{min(valid):.1f}m" if valid else "--"
                avoid_warn = " ⚠️  AVOID ACTIVE" if valid and min(valid) < TEST_AVOID_MARGIN else ""
                print(f"  F:{front}  R:{right}  B:{back}  L:{left}  | Closest:{closest} | Mode:{mode}{avoid_warn}")
            else:
                if msg:
                    dist_m = msg.current_distance / 100.0
                    warn = " ⚠️  AVOID ACTIVE" if dist_m < TEST_AVOID_MARGIN else ""
                    print(f"  Rangefinder: {dist_m:.2f}m | Mode:{mode}{warn}")

            time.sleep(0.5)

    except KeyboardInterrupt:
        log.info("")
        log.info("Ctrl+C received — restoring production avoidance parameters...")
        set_param('AVOID_BEHAVE',    PROD_AVOID_BEHAVE)
        set_param('AVOID_MARGIN',    PROD_AVOID_MARGIN)
        set_param('AVOID_BACKUP_SPD', PROD_AVOID_BACKUP)
        log.info(f"  AVOID_BEHAVE    = {PROD_AVOID_BEHAVE}  (STOP mode restored)")
        log.info(f"  AVOID_MARGIN    = {PROD_AVOID_MARGIN}m")
        log.info(f"  AVOID_BACKUP_SPD= {PROD_AVOID_BACKUP} m/s")
        log.info("Production parameters restored. Land the drone safely!")


if __name__ == '__main__':
    main()
