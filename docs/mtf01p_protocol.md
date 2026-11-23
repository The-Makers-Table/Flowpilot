# MicoAir MTF-01P Optical Flow & ToF Sensor — Protocol Reference

This document summarizes the serial protocols and binary frame formats used by the **MicoAir MTF-01P** optical flow + time-of-flight (ToF) rangefinder modules on the Raspberry Pi 5.

## Physical Connections

| Sensor | Raspberry Pi 5 UART | Baud Rate | Role |
|---|---|---|---|
| **Downward MTF-01P** | `/dev/ttyAMA0` (UART0) | `115200` (8N1) | Ground AGL height + 2D optical flow velocity |
| **Upward MTF-01P** | `/dev/ttyAMA2` (UART2) | `115200` (8N1) | Ceiling clearance distance |

Use [`src/tools/test_optical_flow.py`](../src/tools/test_optical_flow.py) to auto-detect which output mode your MTF-01P is currently configured for (`MicoLink`, `MAVLink v1/v2`, or `MSP`).

---

## Mode 1: `mav_apm` (MAVLink Output — Active Default in `src/start_drone.py`)

Configured via **MicoAssistant** (`mav_apm` mode). The sensor natively emits MAVLink frames at `115200` baud:
- `OPTICAL_FLOW` / `OPTICAL_FLOW_RAD` — Optical flow velocity & quality
- `DISTANCE_SENSOR` — ToF range in centimeters

`src/start_drone.py` forwards the downward sensor's MAVLink byte stream directly to the flight controller (`mtf_mavlink_relay_thread`), where ArduPilot's `FLOW_TYPE=5` and `RNGFND1_TYPE=10` backends fuse them into **EKF3**.

---

## Mode 2: MicoLink Binary Protocol (`0xEF` — Parsed by `src/tools/mtf_raw.py`)

Default factory protocol: fixed **27-byte** frame (`6-byte header + 20-byte payload + 1-byte checksum`).

| Offset | Field | Type | Value / Description |
|---|---|---|---|
| `0` | `header` | `uint8` | `0xEF` |
| `1` | `dev_id` | `uint8` | `0x0F` |
| `2` | `sys_id` | `uint8` | `0x00` |
| `3` | `msg_id` | `uint8` | `0x51` (combined range + flow) |
| `4` | `seq` | `uint8` | Rolling sequence number |
| `5` | `payload_len` | `uint8` | `20` (`0x14`) |
| `6..25` | `payload` | `<IIBBBBhhBBH` | `time_ms`, `dist_mm`, `strength`, `precision`, `dis_status`, `reserved`, `flow_x`, `flow_y`, `flow_quality`, `flow_status`, `reserved` |
| `26` | `checksum` | `uint8` | `sum(frame[0:26]) & 0xFF` |

Diagnostic command:
```bash
python3 src/tools/mtf_raw.py --port /dev/ttyAMA0 --duration 10
```

---

## Mode 3: Interleaved `0x58 0x3C` Binary Stream (Parsed by `parse_mtf_stream` in `src/start_drone.py`)

Interleaved two-frame binary format with header `0x58 0x3C` (`X<`) and footer `0x24` (`$`):

- **Range Frame (`MSG_ID = 0x01`, 14 bytes):** `quality` at byte `7` (`0–255`), `range_mm` at bytes `8..11` (`<I`, little-endian `uint32`).
- **Optical Flow Frame (`MSG_ID = 0x02`, 18 bytes):** `seq` at byte `7`, `flow_x` (`<i`, mm/s) at bytes `8..11`, `flow_y` (`<i`, mm/s) at bytes `12..15`.
