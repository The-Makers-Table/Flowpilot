#!/usr/bin/env python3
"""
arm_disarm.py — Simple Arm / Disarm script for Cube Orange (ArduCopter)
========================================================================
Usage:
  python3 arm_disarm.py arm      → Arms the drone
  python3 arm_disarm.py disarm   → Disarms the drone
  python3 arm_disarm.py status   → Shows current arm status
  python3 arm_disarm.py          → Interactive menu
"""

import os
import sys
import time
import logging
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────────────────────
FC_PORT = os.environ.get('FC_SERIAL_PORT', '/dev/ttyAMA0')
FC_BAUD = int(os.environ.get('FC_BAUD_RATE', '921600'))
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)


def connect():
    log.info(f"Connecting to FC on {FC_PORT}...")
    mav = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
    mav.wait_heartbeat(timeout=10)
    log.info(f"Connected! System={mav.target_system}")
    return mav


def is_armed(mav):
    msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    if msg:
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return None


def get_status(mav):
    armed = is_armed(mav)
    msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    alt = msg.relative_alt / 1000.0 if msg else 0.0

    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    mode = "UNKNOWN"
    if hb:
        mode_map = {v: k for k, v in mav.mode_mapping().items()}
        mode = mode_map.get(hb.custom_mode, str(hb.custom_mode))

    print()
    print("=" * 35)
    print(f"  Armed  : {'✅ YES' if armed else '❌ NO'}")
    print(f"  Mode   : {mode}")
    print(f"  Alt    : {alt:.2f} m")
    print("=" * 35)
    print()


def arm(mav, timeout=15):
    if is_armed(mav):
        log.info("Already armed!")
        return True

    log.info("Sending ARM command...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,      # 1 = arm
        0, 0, 0, 0, 0, 0
    )

    deadline = time.time() + timeout
    while time.time() < deadline:
        # Print any FC messages (pre-arm failures etc.)
        msg = mav.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            log.info(f"FC: {msg.text.strip()}")

        if is_armed(mav):
            log.info("✅ ARMED successfully!")
            return True
        time.sleep(0.3)

    log.error("❌ Arm failed! Check Mission Planner for pre-arm errors.")
    log.error("Common causes: EKF not ready, barometer not calibrated, RC not connected.")
    return False


def disarm(mav, timeout=10, force=False):
    if not is_armed(mav):
        log.info("Already disarmed!")
        return True

    log.info("Sending DISARM command...")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,              # 0 = disarm
        21196 if force else 0,   # 21196 = force disarm magic number
        0, 0, 0, 0, 0
    )

    deadline = time.time() + timeout
    while time.time() < deadline:
        if not is_armed(mav):
            log.info("✅ DISARMED successfully!")
            return True
        time.sleep(0.3)

    log.error("❌ Disarm failed. If drone is in air, land first.")
    return False


def interactive_menu(mav):
    while True:
        print()
        armed = is_armed(mav)
        status_str = "ARMED ✅" if armed else "DISARMED ❌"
        print(f"  Current state: {status_str}")
        print()
        print("  [1] ARM")
        print("  [2] DISARM")
        print("  [3] FORCE DISARM  (use only on ground!)")
        print("  [4] Status")
        print("  [q] Quit")
        print()

        choice = input("  Choice: ").strip().lower()

        if choice == '1':
            arm(mav)
        elif choice == '2':
            disarm(mav)
        elif choice == '3':
            confirm = input("  ⚠️  Force disarm — are you sure? (yes/no): ").strip().lower()
            if confirm == 'yes':
                disarm(mav, force=True)
        elif choice == '4':
            get_status(mav)
        elif choice == 'q':
            print("  Bye!")
            break
        else:
            print("  Invalid choice.")


def main():
    mav = connect()

    if len(sys.argv) < 2:
        interactive_menu(mav)
    else:
        cmd = sys.argv[1].lower()
        if cmd == 'arm':
            arm(mav)
        elif cmd == 'disarm':
            disarm(mav)
        elif cmd == 'status':
            get_status(mav)
        else:
            print(f"Unknown command: {cmd}")
            print("Usage: python3 arm_disarm.py [arm|disarm|status]")
            sys.exit(1)


if __name__ == '__main__':
    main()
