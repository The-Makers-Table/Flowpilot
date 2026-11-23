# Web GCS Dashboards — Architecture Overview

This repository includes **two zero-build, pure-Python web dashboards** built with **FastAPI + WebSockets**, served directly from the Raspberry Pi on port `8765` (`http://<PI_IP>:8765`).

---

## 1. Full 3D Telemetry & LiDAR Dashboard (`src/dashboard/bridge.py` + `index.html`)

Designed to run **alongside** `src/start_drone.py` without competing for the flight controller's serial port.

### Data Flow

```text
SpeedyBee F405 Mini (/dev/ttyAMA4)
        │
        ▼
  src/start_drone.py
   ├── TCP MAVLink Bridge (127.0.0.1:5760) ──────► src/dashboard/bridge.py
   ├── /tmp/drone_obstacles.bin (72×uint16, 10 Hz) ┘   (FastAPI + WebSocket /ws @ 10 Hz)
   └── /tmp/drone_sensors.json (floor/ceil/flow)   ┘           │
                                                               ▼
                                                    Browser (index.html + Three.js)
```

### Capabilities
- **3D Viewport (Three.js r128 + OrbitControls):** Live attitude (roll/pitch/yaw), local NED position, and real-time 360° RPLidar obstacle ring.
- **Telemetry Sidebar:** Armed state, flight mode, battery voltage/percentage (4S LiPo scale `13.2–16.8 V`), floor/ceiling ToF distances, optical flow quality, and scrolling FC `STATUSTEXT` log.
- **Flight Controls:** Mode switching (`STABILIZE`, `ALT_HOLD`, `GUIDED`, `GUIDED_NOGPS`, `LOITER`, `LAND`, `RTL`), Arm/Disarm, and full GPS-denied GUIDED takeoff (injects fake EKF origin → `GUIDED` → Arm → `MAV_CMD_NAV_TAKEOFF`).

### Run
```bash
# Terminal 1 — Sensor bridge daemon
python3 src/start_drone.py

# Terminal 2 — 3D GCS dashboard
python3 src/dashboard/bridge.py
```

---

## 2. Minimal Auto-Loiter Control Dashboard (`src/dashboard/drone_server.py`)

A self-contained, single-file FastAPI + embedded mobile-friendly HTML dashboard that connects **directly** to the flight controller serial port (`/dev/ttyAMA4` by default).

> [!NOTE]
> Because `drone_server.py` opens the flight controller UART directly, do **not** run it simultaneously with `src/start_drone.py` on the same serial port.

### Capabilities
- **One-Tap Takeoff & Auto-Loiter:** Sets fake EKF origin `(0, 0, 0)` → switches to `GUIDED` → arms → climbs to slider target altitude (`0.3–3.0 m`) → monitors `RANGEFINDER` / `GLOBAL_POSITION_INT` in a background thread → automatically switches to `LOITER` once within `±0.10 m`.
- **Live EKF Health Flags:** Displays `ATT`, `VEL`, `POS`, and `HGT` bits from `EKF_STATUS_REPORT`.
- **Emergency Force-Disarm:** Sends `MAV_CMD_COMPONENT_ARM_DISARM` with override magic `21196`.

### Run
```bash
python3 src/dashboard/drone_server.py
```

Override defaults via environment variables:
```bash
FC_SERIAL_PORT=/dev/ttyAMA4 FC_BAUD_RATE=921600 DASHBOARD_PORT=8765 python3 src/dashboard/drone_server.py
```
