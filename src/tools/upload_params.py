#!/usr/bin/env python3
"""
upload_params.py — Upload ArduPilot parameters from the Raspberry Pi
=====================================================================
Reads ardupilot_params.param, connects to the FC over /dev/ttyAMA4
at 921600 baud, and writes every parameter with the correct MAVLink type.

Strategy (avoids the INT8/REAL32 type-mismatch crash):
  1.  Request the full parameter list from the FC and cache every param's
      native MAVLink type (INT8, INT16, INT32, REAL32, etc.).
  2.  Write each target param using the *FC-reported type*, not a guessed type.
  3.  Verify the written value by requesting the param back and comparing.
  4.  Retry up to MAX_RETRIES times for any param that doesn't verify.
  5.  Print a final summary of OK / FAILED / SKIPPED params.
  6.  Optionally reboot the FC when done (--reboot flag).

Usage
─────
  python3 upload_params.py                  # upload + ask about reboot
  python3 upload_params.py --reboot         # upload + auto-reboot
  python3 upload_params.py --dry-run        # parse .param file, don't write anything
  python3 upload_params.py --verify-only    # read FC params and diff against .param file
"""

import argparse
import logging
import os
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

PARAM_FILE   = Path(__file__).resolve().parent.parent.parent / "firmware_config" / "ardupilot_params.param"
FC_PORT      = os.environ.get("FC_SERIAL_PORT", "/dev/ttyAMA4")
FC_BAUD      = int(os.environ.get("FC_BAUD_RATE", "921600"))
MAX_RETRIES  = 3          # per-parameter retry attempts
WRITE_TIMEOUT_S = 3.0    # seconds to wait for PARAM_VALUE echo after a write
FETCH_TIMEOUT_S = 60.0   # seconds to wait for full parameter list download

# ─────────────────────────────────────────────────────────────────────────────
# LOGGING
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("upload_params")


# ─────────────────────────────────────────────────────────────────────────────
# PARAM FILE PARSER
# ─────────────────────────────────────────────────────────────────────────────

def parse_param_file(path: Path) -> Dict[str, float]:
    """
    Parse a Mission Planner .param file into {name: value} dict.

    Supported line formats:
      PARAM_NAME,VALUE
      PARAM_NAME,VALUE   # inline comment
      # full-line comment  (skipped)
      blank lines         (skipped)
    """
    params: Dict[str, float] = {}
    skipped = 0

    with open(path, "r") as f:
        for lineno, raw_line in enumerate(f, start=1):
            line = raw_line.strip()

            # Skip blank lines and full-line comments
            if not line or line.startswith("#"):
                continue

            # Strip inline comment
            if "#" in line:
                line = line[:line.index("#")].strip()

            # Parse NAME,VALUE
            parts = line.split(",", 1)
            if len(parts) != 2:
                log.warning(f"Line {lineno}: cannot parse — '{raw_line.rstrip()}'")
                skipped += 1
                continue

            name  = parts[0].strip().upper()
            value_str = parts[1].strip()

            try:
                value = float(value_str)
            except ValueError:
                log.warning(f"Line {lineno}: non-numeric value '{value_str}' for {name} — skipping")
                skipped += 1
                continue

            params[name] = value

    log.info(f"Parsed {len(params)} parameters from {path.name}  ({skipped} lines skipped)")
    return params


# ─────────────────────────────────────────────────────────────────────────────
# FC CONNECTION
# ─────────────────────────────────────────────────────────────────────────────

def connect_fc() -> mavutil.mavfile:
    """Open MAVLink connection and wait for heartbeat."""
    log.info(f"Connecting to FC on {FC_PORT} @ {FC_BAUD} baud ...")
    mav = mavutil.mavlink_connection(
        FC_PORT,
        baud=FC_BAUD,
        source_system=255,
        source_component=0,
    )
    log.info("Waiting for FC heartbeat (timeout 15 s) ...")
    hb = mav.wait_heartbeat(timeout=15)
    if hb is None:
        log.critical("No heartbeat received. Check wiring and that FC is powered.")
        sys.exit(1)
    log.info(f"FC heartbeat received  →  sys={mav.target_system}  comp={mav.target_component}")
    return mav


# ─────────────────────────────────────────────────────────────────────────────
# FETCH ALL FC PARAMETERS  (to get native MAVLink types)
# ─────────────────────────────────────────────────────────────────────────────

def fetch_all_params(mav: mavutil.mavfile) -> Dict[str, Tuple[float, int]]:
    """
    Request the full parameter list from the FC.

    Returns:
        dict of { param_name: (current_value, mavlink_param_type) }
        where mavlink_param_type is a MAV_PARAM_TYPE_* integer.
    """
    log.info("Requesting full parameter list from FC ...")
    mav.mav.param_request_list_send(mav.target_system, mav.target_component)

    fc_params: Dict[str, Tuple[float, int]] = {}
    total_expected = None
    deadline = time.monotonic() + FETCH_TIMEOUT_S
    last_progress = time.monotonic()

    while time.monotonic() < deadline:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        if msg is None:
            # Re-request if we've been waiting a while with no new params
            if time.monotonic() - last_progress > 10.0:
                log.warning("No params received for 10 s — re-requesting list ...")
                mav.mav.param_request_list_send(mav.target_system, mav.target_component)
                last_progress = time.monotonic()
            continue

        name  = msg.param_id.rstrip("\x00").strip()
        value = float(msg.param_value)
        ptype = int(msg.param_type)
        count = int(msg.param_count)

        fc_params[name] = (value, ptype)
        last_progress = time.monotonic()

        if total_expected is None:
            total_expected = count
            log.info(f"FC reports {total_expected} total parameters")

        # Progress every 50 params
        n = len(fc_params)
        if n % 50 == 0:
            log.info(f"  Downloaded {n}/{total_expected} ...")

        if total_expected and len(fc_params) >= total_expected:
            break

    if total_expected and len(fc_params) < total_expected:
        log.warning(
            f"Only received {len(fc_params)}/{total_expected} params before timeout. "
            "Proceeding with what we have."
        )
    else:
        log.info(f"Downloaded all {len(fc_params)} parameters ✓")

    return fc_params


# ─────────────────────────────────────────────────────────────────────────────
# WRITE A SINGLE PARAMETER
# ─────────────────────────────────────────────────────────────────────────────

def write_param(mav: mavutil.mavfile, name: str, value: float,
                ptype: int) -> Optional[float]:
    """
    Send PARAM_SET and wait for the FC's PARAM_VALUE echo.

    Returns the value confirmed by the FC, or None on timeout.
    """
    # Encode param name as null-padded 16-byte string
    name_encoded = name.encode("ascii").ljust(16, b"\x00")[:16]

    mav.mav.param_set_send(
        target_system=mav.target_system,
        target_component=mav.target_component,
        param_id=name_encoded,
        param_value=float(value),
        param_type=ptype,
    )

    # Wait for the echo — may receive other PARAM_VALUEs first (ignore them)
    deadline = time.monotonic() + WRITE_TIMEOUT_S
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        recv_name = msg.param_id.rstrip("\x00").strip()
        if recv_name == name:
            return float(msg.param_value)

    return None  # timed out


# ─────────────────────────────────────────────────────────────────────────────
# VALUE COMPARISON  (float equality with tolerance)
# ─────────────────────────────────────────────────────────────────────────────

def values_match(a: float, b: float) -> bool:
    """
    Consider two parameter values equal if they're within 0.01% of each other
    OR within a tiny absolute epsilon.  ArduPilot sometimes stores floats with
    slight rounding, so exact == comparison fails for values like 2.0 stored as
    1.9999998... in REAL32.
    """
    if a == b:
        return True
    # Absolute tolerance for near-zero values
    if abs(a) < 1e-6 and abs(b) < 1e-6:
        return True
    # Relative tolerance
    return abs(a - b) / max(abs(a), abs(b), 1e-9) < 1e-4


# ─────────────────────────────────────────────────────────────────────────────
# REBOOT FC
# ─────────────────────────────────────────────────────────────────────────────

def save_params(mav: mavutil.mavfile) -> None:
    """Send the preflight storage command to the FC to save parameters."""
    log.info("Sending command to save parameters to EEPROM...")
    mav.mav.command_long_send(
        target_system=mav.target_system,
        target_component=mav.target_component,
        command=mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
        confirmation=0,
        param1=1,       # 1 = write parameters to storage
        param2=0, param3=0, param4=0,
        param5=0, param6=0, param7=0,
    )
    time.sleep(1.0)

def reboot_fc(mav: mavutil.mavfile) -> None:
    """Send the preflight reboot command to the FC."""
    log.info("Sending reboot command to FC ...")
    mav.mav.command_long_send(
        target_system=mav.target_system,
        target_component=mav.target_component,
        command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        confirmation=0,
        param1=1,       # 1 = reboot autopilot
        param2=0, param3=0, param4=0,
        param5=0, param6=0, param7=0,
    )
    log.info("Reboot command sent. FC will restart in ~5 s.")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    # ── Arguments ─────────────────────────────────────────────────────────────
    parser = argparse.ArgumentParser(
        description="Upload ardupilot_params.param to FC via UART (no Mission Planner needed)"
    )
    parser.add_argument(
        "--reboot", action="store_true",
        help="Automatically reboot FC after successful upload",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Parse the .param file and show what would be written, but don't connect to FC",
    )
    parser.add_argument(
        "--verify-only", action="store_true",
        help="Connect to FC, download params, and show diff against .param file (no writes)",
    )
    parser.add_argument(
        "--param-file", type=Path, default=PARAM_FILE,
        help=f"Path to .param file (default: {PARAM_FILE})",
    )
    args = parser.parse_args()

    # ── Parse .param file ─────────────────────────────────────────────────────
    if not args.param_file.exists():
        log.critical(f"Parameter file not found: {args.param_file}")
        sys.exit(1)

    target_params = parse_param_file(args.param_file)

    if args.dry_run:
        log.info("=== DRY RUN — parameters that would be written ===")
        for name, value in sorted(target_params.items()):
            log.info(f"  {name:<30} = {value}")
        sys.exit(0)

    # ── Connect to FC ─────────────────────────────────────────────────────────
    mav = connect_fc()

    # ── Download FC param list (to get types) ─────────────────────────────────
    fc_params = fetch_all_params(mav)

    # ── Verify-only mode ──────────────────────────────────────────────────────
    if args.verify_only:
        log.info("=== VERIFY MODE — diff between .param file and FC ===")
        match_count    = 0
        mismatch_count = 0
        unknown_count  = 0

        for name, target_val in sorted(target_params.items()):
            if name not in fc_params:
                log.warning(f"  UNKNOWN  {name:<30}  (not in FC param list)")
                unknown_count += 1
                continue
            fc_val, _ = fc_params[name]
            if values_match(fc_val, target_val):
                log.info(f"  OK       {name:<30}  = {fc_val}")
                match_count += 1
            else:
                log.warning(f"  MISMATCH {name:<30}  FC={fc_val}  Target={target_val}")
                mismatch_count += 1

        log.info(
            f"\nSummary: {match_count} match | {mismatch_count} mismatch | {unknown_count} unknown"
        )
        sys.exit(0)

    # ── Upload parameters ─────────────────────────────────────────────────────
    log.info(f"Starting upload of {len(target_params)} parameters ...")

    results = {"ok": [], "failed": [], "skipped": [], "unchanged": []}

    for idx, (name, target_val) in enumerate(target_params.items(), start=1):
        prefix = f"[{idx:>2}/{len(target_params)}]"

        # Check if FC knows this parameter
        if name not in fc_params:
            log.warning(f"{prefix} SKIP     {name:<30} — not in FC param list")
            results["skipped"].append(name)
            continue

        fc_val, ptype = fc_params[name]

        # Skip if already correct
        if values_match(fc_val, target_val):
            log.info(f"{prefix} UNCHANGED {name:<30} = {fc_val} (already correct)")
            results["unchanged"].append(name)
            continue

        log.info(
            f"{prefix} WRITE    {name:<30}  {fc_val}  →  {target_val}  (type={ptype})"
        )

        # Try writing with retries
        confirmed_val = None
        for attempt in range(1, MAX_RETRIES + 1):
            confirmed_val = write_param(mav, name, target_val, ptype)
            if confirmed_val is not None:
                break
            if attempt < MAX_RETRIES:
                log.warning(f"  Attempt {attempt} timed out — retrying ...")
                time.sleep(0.5)

        if confirmed_val is None:
            log.error(f"  FAILED   {name} — no echo after {MAX_RETRIES} attempts")
            results["failed"].append(name)
            continue

        if values_match(confirmed_val, target_val):
            log.info(f"  OK       {name} confirmed = {confirmed_val}")
            results["ok"].append(name)
        else:
            log.error(
                f"  MISMATCH {name} — wrote {target_val} but FC returned {confirmed_val}"
            )
            results["failed"].append(name)

        # Small delay between writes to avoid overwhelming the FC's write queue
        time.sleep(0.05)

    # ── Summary ───────────────────────────────────────────────────────────────
    log.info("")
    log.info("══════════════════════════════════════════════")
    log.info("  PARAMETER UPLOAD COMPLETE")
    log.info("══════════════════════════════════════════════")
    log.info(f"  Written & verified : {len(results['ok'])}")
    log.info(f"  Already correct    : {len(results['unchanged'])}")
    log.info(f"  Skipped (unknown)  : {len(results['skipped'])}")
    log.info(f"  FAILED             : {len(results['failed'])}")

    if results["skipped"]:
        log.warning(f"  Skipped params: {', '.join(results['skipped'])}")
    if results["failed"]:
        log.error(f"  Failed params : {', '.join(results['failed'])}")
        log.error("  These parameters were NOT applied. Check FC logs.")

    # ── Reboot ────────────────────────────────────────────────────────────────
    if results["failed"]:
        log.warning("Upload had failures — skipping reboot. Fix issues and re-run.")
        sys.exit(1)

    do_reboot = args.reboot
    if not do_reboot and results["ok"]:
        # Ask interactively
        try:
            answer = input("\nReboot FC now to apply changes? [Y/n]: ").strip().lower()
            do_reboot = answer in ("", "y", "yes")
        except (EOFError, KeyboardInterrupt):
            do_reboot = False

    if do_reboot:
        save_params(mav)
        reboot_fc(mav)
        log.info("Waiting 8 s for FC to restart ...")
        time.sleep(8.0)
        log.info("Done. FC should be back online.")
    else:
        log.info("Reboot skipped. Power-cycle the FC manually before flying.")

    sys.exit(0 if not results["failed"] else 1)


if __name__ == "__main__":
    main()
