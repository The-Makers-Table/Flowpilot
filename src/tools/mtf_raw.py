#!/usr/bin/env python3
"""
mtf_raw.py — Raw serial diagnostic for MTF-01P MicoLink frames.
Run this standalone (start_drone.py must NOT be running — only one process
can hold a serial port at a time).

Usage:
  python3 mtf_raw.py             # tests /dev/ttyAMA0 (downward sensor, 10 s)
  python3 mtf_raw.py --port /dev/ttyAMA2   # upward sensor

What it does:
  - Reads raw bytes for 10 seconds
  - Reports how many bytes arrived (0 = wiring/power problem)
  - Counts 0xEF header bytes found
  - Prints the first valid MicoLink frame decoded
  - If checksum never matches, dumps the first candidate frame as hex
    so you can verify the checksum algorithm manually
"""

import argparse
import struct
import sys
import time

import serial

BAUD        = 115200
DURATION_S  = 10

HEADER      = 0xEF
DEV_ID      = 0x0F
SYS_ID      = 0x00
MSG_ID      = 0x51
PAYLOAD_LEN = 20           # 0x14
FRAME_LEN   = 27           # 6 header + 20 payload + 1 checksum
PAYLOAD_FMT = "<IIBBBBhhBBH"


def checksum(frame_bytes: bytes) -> int:
    """MicoLink: sum of all bytes before the checksum byte."""
    return sum(frame_bytes[:-1]) & 0xFF


def parse_frame(frame: bytes):
    """Returns decoded dict or None."""
    if len(frame) != FRAME_LEN:
        return None
    if frame[0] != HEADER:
        return None
    if frame[3] != MSG_ID:
        return None
    if frame[5] != PAYLOAD_LEN:
        return None
    expected = checksum(frame)
    actual   = frame[26]
    if expected != actual:
        return {"checksum_error": True, "expected": expected, "actual": actual}
    payload = frame[6:26]
    try:
        (time_ms, dist_mm, strength, precision, dis_status,
         _r1, flow_x, flow_y, flow_quality, flow_status,
         _r2) = struct.unpack(PAYLOAD_FMT, payload)
        return {
            "checksum_error": False,
            "time_ms":    time_ms,
            "dist_mm":    dist_mm,
            "strength":   strength,
            "dis_status": dis_status,
            "flow_x":     flow_x,
            "flow_y":     flow_y,
            "flow_q":     flow_quality,
            "flow_st":    flow_status,
        }
    except struct.error as e:
        return {"struct_error": str(e)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyAMA0",
                        help="Serial port to sniff (default: /dev/ttyAMA0)")
    parser.add_argument("--duration", type=int, default=DURATION_S,
                        help="How many seconds to run (default: 10)")
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print(f"MTF-01P raw serial sniffer")
    print(f"Port:     {args.port}")
    print(f"Baud:     {BAUD}")
    print(f"Duration: {args.duration} s")
    print(f"{'='*60}\n")

    try:
        ser = serial.Serial(args.port, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {args.port}: {e}")
        print("Check wiring and that start_drone.py is NOT running.")
        sys.exit(1)

    buf = bytearray()
    total_bytes   = 0
    header_hits   = 0
    valid_frames  = 0
    bad_checksum  = 0
    first_frame   = None
    first_bad_hex = None

    deadline = time.monotonic() + args.duration

    print(f"Reading for {args.duration} s ... (Ctrl+C to stop early)\n")

    try:
        while time.monotonic() < deadline:
            chunk = ser.read(64)
            if not chunk:
                continue
            total_bytes += len(chunk)
            buf.extend(chunk)

            # Slide through buffer
            while len(buf) >= FRAME_LEN:
                idx = buf.find(HEADER)
                if idx == -1:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < FRAME_LEN:
                    break

                header_hits += 1
                frame = bytes(buf[:FRAME_LEN])
                result = parse_frame(frame)

                if result is None:
                    del buf[0]
                    continue

                if result.get("checksum_error"):
                    bad_checksum += 1
                    if first_bad_hex is None:
                        first_bad_hex = frame.hex()
                        exp = result["expected"]
                        act = result["actual"]
                        print(f"[CHECKSUM FAIL] expected=0x{exp:02X} actual=0x{act:02X}")
                        print(f"  Frame hex: {first_bad_hex}")
                        print(f"  Header bytes: "
                              f"0x{frame[0]:02X} 0x{frame[1]:02X} 0x{frame[2]:02X} "
                              f"0x{frame[3]:02X} 0x{frame[4]:02X} 0x{frame[5]:02X}")
                    del buf[0]
                    continue

                if result.get("struct_error"):
                    print(f"[STRUCT ERROR] {result['struct_error']}")
                    del buf[0]
                    continue

                # Valid frame!
                valid_frames += 1
                del buf[:FRAME_LEN]

                if first_frame is None:
                    first_frame = result
                    print("=" * 50)
                    print("✓ VALID FRAME DECODED!")
                    print(f"  dist_mm    = {result['dist_mm']}  ({'OK' if result['dis_status'] else 'INVALID'})")
                    print(f"  strength   = {result['strength']}")
                    print(f"  flow_x     = {result['flow_x']} cm/s@1m")
                    print(f"  flow_y     = {result['flow_y']} cm/s@1m")
                    print(f"  flow_q     = {result['flow_q']}/255")
                    print(f"  flow_st    = {result['flow_st']}")
                    print("=" * 50)

    except KeyboardInterrupt:
        print("\nStopped early.")
    finally:
        ser.close()

    print(f"\n{'='*60}")
    print("RESULTS")
    print(f"{'='*60}")
    print(f"Total bytes received : {total_bytes}")
    print(f"0xEF header hits     : {header_hits}")
    print(f"Valid frames decoded : {valid_frames}")
    print(f"Checksum failures    : {bad_checksum}")
    print()

    if total_bytes == 0:
        print("❌ NO BYTES RECEIVED. Probable causes:")
        print("   1. Sensor not powered (check 3.3V/5V supply)")
        print("   2. TX/RX wired backwards (swap them)")
        print("   3. Wrong UART port (try --port /dev/ttyAMA2)")
        print("   4. UART not enabled in /boot/firmware/config.txt")
    elif header_hits == 0:
        print("❌ Bytes arriving but no 0xEF headers found.")
        print("   This sensor is probably outputting a DIFFERENT protocol.")
        print(f"   First 32 bytes: {bytes(buf[:32]).hex() if buf else '<buffer empty>'}")
    elif valid_frames == 0 and bad_checksum > 0:
        print("❌ Headers found but checksum always fails.")
        print("   The checksum algorithm may still be wrong.")
        print("   Check the first_bad_hex above and compute manually:")
        print("   sum(frame[0:26]) & 0xFF  should equal frame[26]")
    elif valid_frames > 0:
        print("✓ Parser is working. Sensor is healthy.")
        print(f"  Frame rate ≈ {valid_frames / args.duration:.1f} Hz")
    print()


if __name__ == "__main__":
    main()
