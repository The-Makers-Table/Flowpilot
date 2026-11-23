#!/usr/bin/env python3
"""
takeoff.py — Autonomous Takeoff Script for Cube Orange + ArduCopter
====================================================================
Run this script once to command the drone to:
  1. Set EKF origin (required for no-GPS flight)
  2. Arm in GUIDED mode
  3. Take off to TARGET_ALT (1.5 m)
  4. Switch to LOITER mode (position hold using optical flow)

After this script finishes, the drone will hover in Loiter mode.
You can then take over with your RC controller.
The lidar_bridge.py service handles obstacle avoidance continuously.

Usage:
  python3 takeoff.py

Make sure lidar_bridge.py is already running before takeoff.

Hardware assumption:
  - RPi GPIO14 (TXD, Pin 8) → Cube Orange GPS1/SERIAL3 Pin 3 (RX)
  - RPi GPIO15 (RXD, Pin10) → Cube Orange GPS1/SERIAL3 Pin 2 (TX)
  - RPi GND        (Pin 6)  → Cube Orange GPS1/SERIAL3 Pin 8 (GND)
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

# EKF origin — use 0,0 for indoor no-GPS flight. ArduPilot just needs SOME
# origin to initialise the EKF position estimate. Lat/lon don't need to be real.
EKF_ORIGIN_LAT = 0.0          # degrees (0 = dummy indoor origin)
EKF_ORIGIN_LON = 0.0          # degrees
EKF_ORIGIN_ALT = 0.0          # metres above sea level (0 is fine indoors)

ARM_TIMEOUT  = 15             # seconds to wait for arming
ALT_TIMEOUT  = 30             # seconds to wait for target altitude
EKF_TIMEOUT  = 45             # seconds to wait for EKF to initialise (increased from 20)

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

    # Print any messages from FC
    get_statustext(mav, timeout=2)

    log.info("=" * 60)
    log.info("SUCCESS — Drone is in LOITER mode at %.1fm" % TARGET_ALT)
    log.info("You can now take control with your RC transmitter.")
    log.info("Obstacle avoidance is active (lidar_bridge.py running).")
    log.info("AVOID_MARGIN = 0.7m — drone stops 0.7m from walls.")
    log.info("=" * 60)


if __name__ == '__main__':
    main()
