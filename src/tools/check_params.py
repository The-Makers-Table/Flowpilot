#!/usr/bin/env python3
"""
check_params.py — Search the FC parameter list for known param variants.
Prints what names the FC actually uses for flow, proximity, and safety params.
Run this AFTER upload_params.py has run at least once and FC has rebooted.
"""

import sys
from pathlib import Path
from pymavlink import mavutil

# Ensure src/ is on sys.path when invoked from repo root or src/tools/
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import config

SEARCH_TERMS = [
    "FLOW", "PRX", "OA_", "BRD_SAFE", "BRD_OPTIONS",
    "EK3_GPS", "RNGFND_LAND", "TERRAIN", "RC_FEEL",
    "FENCE", "AVOID",
]

print(f"Connecting to FC on {config.FC_SERIAL_PORT} @ {config.FC_BAUD_RATE}...")
mav = mavutil.mavlink_connection(config.FC_SERIAL_PORT, baud=config.FC_BAUD_RATE,
                                  source_system=255, source_component=0)
hb = mav.wait_heartbeat(timeout=15)
if not hb:
    print("No heartbeat"); sys.exit(1)
print(f"Connected. Downloading params...")

mav.mav.param_request_list_send(mav.target_system, mav.target_component)

found: dict[str, float] = {}
total = None
import time
deadline = time.monotonic() + 90
while time.monotonic() < deadline:
    msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
    if not msg:
        continue
    name = msg.param_id.rstrip("\x00").strip()
    found[name] = float(msg.param_value)
    if total is None:
        total = msg.param_count
    if total and len(found) >= total:
        break

print(f"\nDownloaded {len(found)} params. Searching for relevant ones:\n")
print(f"{'PARAM NAME':<35} {'VALUE'}")
print("-" * 50)
for term in SEARCH_TERMS:
    matches = {k: v for k, v in found.items() if term.upper() in k.upper()}
    if matches:
        print(f"\n  ── {term} ──")
        for k, v in sorted(matches.items()):
            print(f"  {k:<33} {v}")
    else:
        print(f"\n  ── {term} ──  (no matches)")
