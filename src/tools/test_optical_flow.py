#!/usr/bin/env python3
"""
test_optical_flow.py — Quick serial protocol auto-detector for the MTF-01P sensor.

Reads a short burst of bytes from the configured UART and inspects header bytes
to identify whether the sensor is outputting MicoLink (0xEF), MAVLink v1/v2
(0xFE / 0xFD), or MSP (0x24 / '$').

Usage:
    python3 src/tools/test_optical_flow.py
    python3 src/tools/test_optical_flow.py --port /dev/ttyAMA2 --baud 115200
"""

import argparse
import os
import time
import serial

DEFAULT_PORT = os.environ.get("MTF_DOWN_PORT", "/dev/ttyAMA0")
DEFAULT_BAUD = int(os.environ.get("MTF_DOWN_BAUD", "115200"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect MTF-01P serial output protocol")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial device (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    args = parser.parse_args()

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,
        )

        print(f"Opening {args.port} @ {args.baud}... Please lift or tilt the sensor to generate tracking data.")
        time.sleep(0.5)

        ser.reset_input_buffer()
        raw_data = ser.read(30)
        ser.close()

        if not raw_data:
            print("\n[!] No data received. Troubleshoot steps:")
            print("  1. Ensure the MTF-01P TX pin is wired to Pi Pin 10 (GPIO 15 / RXD0).")
            print("  2. Ensure the sensor is powered via a stable 5V and shares a common GND.")
            print("  3. Double check if the Pi's serial console login is disabled (conflicts with ttyAMA0).")
            return

        hex_string = " ".join(f"0x{b:02X}" for b in raw_data[:10])
        print("\nSuccessfully read data stream!")
        print(f"First 10 Bytes: {hex_string}")

        detected = False
        for byte in raw_data:
            if byte == 0xEF:
                print("\n>>> DETECTED PROTOCOL: MICOLINK <<<")
                print("Notes: This is MicoAir's lightweight, proprietary protocol.")
                detected = True
                break
            elif byte in (0xFE, 0xFD):
                version = "v1" if byte == 0xFE else "v2"
                print(f"\n>>> DETECTED PROTOCOL: MAVLINK ({version}) <<<")
                print("Notes: Commonly used for ArduPilot or PX4 systems.")
                detected = True
                break
            elif byte == 0x24:  # ASCII '$'
                print("\n>>> DETECTED PROTOCOL: MSP (MultiWii Serial Protocol) <<<")
                print("Notes: Usually standard if the unit was pre-flashed for iNav/Betaflight.")
                detected = True
                break

        if not detected:
            print("\nHeader pattern unrecognized. Run the script once more to verify.")

    except serial.SerialException as se:
        print(f"\n[!] Serial Error: {se}")
        print("Hint: Ensure your user is in the 'dialout' group or run with sudo.")
    except Exception as exc:
        print(f"\nAn error occurred: {exc}")


if __name__ == "__main__":
    main()
