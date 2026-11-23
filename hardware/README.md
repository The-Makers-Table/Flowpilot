# Hardware Architecture & Wiring Reference

This repository supports **two physical indoor drone configurations**, both built around a **Raspberry Pi companion computer**, **MicoAir MTF-01P optical flow + ToF rangefinder**, and **RPLidar A1/A1M8 2D LiDAR** running **ArduCopter (ArduPilot)** with no GPS.

---

## Configuration A — Headless Autonomous Stack (SpeedyBee F405 Mini + Raspberry Pi 5)

Used by `src/start_drone.py`, `src/auto_navigate.py`, `src/dashboard/bridge.py`, and `src/dashboard/drone_server.py`.

### Bill of Materials (BOM)

| Component | Interface / Port | Role |
|---|---|---|
| **SpeedyBee F405 Mini** | `SERIAL2` @ `921600` baud | Flight Controller (ArduCopter 4.6.2 / 4.7.0, Quad-X, no GPS, no compass) |
| **Raspberry Pi 5** | `/dev/ttyAMA4` (UART4) | Companion computer (sensor bridge, autonomy, web GCS) |
| **MicoAir MTF-01P (Downward)** | `/dev/ttyAMA0` (UART0) @ `115200` | Optical flow velocity + floor ToF distance (`mav_apm` or MicoLink mode) |
| **MicoAir MTF-01P (Upward)** | `/dev/ttyAMA2` (UART2) @ `115200` | Ceiling clearance ToF rangefinder |
| **Slamtec RPLidar A1** | `/dev/rplidar` (USB udev symlink) @ `115200` | 360° 2D obstacle scan (~8 Hz) |
| **5V BEC (5.1–5.2 V, ≥ 5 A)** | RPi GPIO Pins 2 & 4 (+5V), 6 & 9 (GND) | Regulated companion power (prevents RPi 5 PMIC brownouts) |

### Raspberry Pi 5 UART Overlays (`/boot/firmware/config.txt`)

```ini
# UART4 — Flight Controller MAVLink 2 link (/dev/ttyAMA4)
dtoverlay=uart4

# UART0 — Downward MTF-01P (/dev/ttyAMA0), frees PL011 UART from Bluetooth
dtoverlay=disable-bt

# UART2 — Upward MTF-01P (/dev/ttyAMA2)
dtoverlay=uart2
```

### RPLidar Persistent USB Symlink (`/etc/udev/rules.d/99-rplidar.rules`)

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
```

---

## Configuration B — RC-Assisted Position-Hold + Proximity Avoidance (Cube Orange + Raspberry Pi 4/5)

Used by `src/position_hold/` (`lidar_bridge.py`, `takeoff.py`, `takeoff_avoidance_test.py`, `arm_disarm.py`).

### Bill of Materials (BOM)

| Component | Interface / Port | Role |
|---|---|---|
| **Cube Orange** | `SERIAL4` (GPS2) @ `921600`, `SERIAL2` (TELEM2) @ `115200` | Flight Controller (ArduCopter) |
| **Raspberry Pi 4 / 5** | `/dev/ttyAMA0` (GPIO 14 TX / GPIO 15 RX) | Companion computer running `lidar_bridge.py` |
| **MicoAir MTF-01P** | Wired directly to Cube Orange `SERIAL2` (TELEM2) | Optical flow + downward rangefinder (MAVLink mode) |
| **Slamtec RPLidar A1M8** | `/dev/ttyUSB0` (USB) @ `115200` | 360° proximity avoidance (`OBSTACLE_DISTANCE`) |
| **RC Transmitter** | Spring-center throttle (`PILOT_THR_BHV=7`) | Manual flight in `LOITER` with wall stop/slide protection |

### Cube Orange GPS2 ↔ Raspberry Pi Wiring (3.3 V Logic)

```text
Cube Orange GPS2 (JST-GH 6-pin)     Raspberry Pi 40-pin GPIO
─────────────────────────────────    ────────────────────────────
Pin 1: 5V    (DO NOT CONNECT)        —
Pin 2: TX  ──────────────────────►  Pin 10  (GPIO 15 / RXD)
Pin 3: RX  ◄──────────────────────  Pin  8  (GPIO 14 / TXD)
Pin 4: SCL   (DO NOT CONNECT)        —
Pin 5: SDA   (DO NOT CONNECT)        —
Pin 6: GND ──────────────────────►  Pin  6  (GND)
```

### Raspberry Pi 5 Power Best Practice

> [!IMPORTANT]
> Supply **5.1–5.2 V** from a ≥ 5 A BEC and connect **both** 5V pins (Pins 2 & 4) and **two** GND pins (Pins 6 & 9) using 20 AWG or thicker wire to avoid RPi 5 PMIC under-voltage resets during takeoff current spikes.

---

## Assets to Add Manually

This directory is ready for your mechanical and electrical design files:
- `hardware/cad/` — Frame STL/STEP files, sensor mounts (MTF-01P downward/upward brackets, RPLidar top plate, RPi 5 standoff mount)
- `hardware/wiring_diagram.png` — Schematic / Fritzing / KiCad wiring diagram
