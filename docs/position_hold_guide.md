# Position Hold + Obstacle Avoidance Drone
### Cube Orange · RPLidar A1M8 · MTF-01P Optical Flow · Raspberry Pi

A fully autonomous indoor drone system with:
- **GPS-free position hold** using MTF-01P optical flow sensor
- **360° obstacle avoidance** using RPLidar A1M8 over MAVLink
- **Autonomous takeoff** triggered from Raspberry Pi
- **RC transmitter control** with wall protection (drone stops/slides before hitting walls)

---

## 📦 Hardware Used

| Component | Purpose |
|---|---|
| **Cube Orange** | Flight controller (ArduCopter firmware) |
| **Raspberry Pi** (4 or 5) | Companion computer |
| **RPLidar A1M8** | 360° obstacle avoidance (USB → RPi) |
| **MTF-01P Optical Flow Sensor** | GPS-free position hold (MAVLink → FC SERIAL2) |
| **RC Transmitter** | Manual control (spring-center throttle) |
| **5V 5.1–5.2V BEC** | Power Raspberry Pi (5A minimum) |

---

## 🔌 Wiring

### 1. Raspberry Pi Power (from BEC)
> ⚠️ Use 5.1–5.2V output — 5.0V will cause PMIC under-voltage resets on RPi 5.
> Connect BOTH 5V pins for better current distribution.

```
BEC +5V  ──►  RPi Pin 2  (5V)
BEC +5V  ──►  RPi Pin 4  (5V)   ← connect both for lower resistance
BEC GND  ──►  RPi Pin 6  (GND)
BEC GND  ──►  RPi Pin 9  (GND)  ← connect both
```

### 2. RPi → Cube Orange GPS2 Port (SERIAL4) — MAVLink
> Only 3 wires needed. Both sides are 3.3V logic — no level shifter required.
> GPS2 port is a JST-GH 6-pin connector.

```
Cube Orange GPS2 (JST-GH 6-pin)     Raspberry Pi 40-pin GPIO
─────────────────────────────────    ────────────────────────────
Pin 1: 5V    ← DO NOT CONNECT        —
Pin 2: TX  ──────────────────────►  Pin 10  (GPIO 15 / RXD)
Pin 3: RX  ◄──────────────────────  Pin  8  (GPIO 14 / TXD)
Pin 4: SCL ← DO NOT CONNECT          —
Pin 5: SDA ← DO NOT CONNECT          —
Pin 6: GND ──────────────────────►  Pin  6  (GND)
```

### 3. RPLidar A1M8 → Raspberry Pi
```
RPLidar A1M8 USB adapter → any RPi USB port
Appears as: /dev/ttyUSB0
```

### 4. MTF-01P → Cube Orange SERIAL2 (TELEM2)
```
MTF-01P → Cube Orange TELEM2 port (SERIAL2)
Protocol: MAVLink (set via MicoAssistant app)
```

---

## ⚙️ ArduPilot Parameter Configuration

Set all parameters in **Mission Planner → Full Parameter List**, then reboot the FC.

### MTF-01P Optical Flow (SERIAL2)

| Parameter | Value | Notes |
|---|---|---|
| `SERIAL2_PROTOCOL` | `1` | MAVLink |
| `SERIAL2_BAUD` | `115` | 115200 baud |
| `SERIAL2_OPTIONS` | `1024` | Don't forward MAVLink |
| `FLOW_TYPE` | `5` | MAVLink optical flow |
| `RNGFND1_TYPE` | `10` | MAVLink rangefinder |
| `RNGFND1_MAX` | `8` | Max range 8m |
| `RNGFND1_ORIENT` | `25` | Downward facing |

### EKF3 — No GPS (optical flow only)

| Parameter | Value | Notes |
|---|---|---|
| `EK3_SRC1_POSXY` | `0` | No GPS |
| `EK3_SRC1_VELXY` | `5` | Optical Flow |
| `EK3_SRC1_POSZ` | `1` | Barometer |
| `EK3_SRC1_VELZ` | `0` | None |
| `EK3_SRC1_YAW` | `1` | Compass |
| `EK3_SRC_OPTIONS` | `0` | — |
| `GPS1_TYPE` | `0` | Disable GPS |
| `AHRS_OPTIONS` | `24` | Save/restore EKF origin across reboots |

### Arming

| Parameter | Value | Notes |
|---|---|---|
| `ARMING_CHECK` | Remove GPS bit | Uncheck GPS in pre-arm checks |

### RPi MAVLink (SERIAL4 / GPS2 port)

| Parameter | Value | Notes |
|---|---|---|
| `SERIAL4_PROTOCOL` | `2` | MAVLink2 |
| `SERIAL4_BAUD` | `921600` | High baud rate |

> Note: GPS2 port has no RTS/CTS pins, so `BRD_SER4_RTSCTS` does not exist — flow control is already disabled.

### RPLidar Proximity Avoidance via MAVLink

| Parameter | Value | Notes |
|---|---|---|
| `PRX1_TYPE` | `2` | MAVLink proximity — required! |
| `PRX1_ORIENT` | `0` | Horizontal |
| `AVOID_ENABLE` | `3` | Proximity + fence avoidance |
| `AVOID_BEHAVE` | `1` | **Stop** mode — drone stops at wall |
| `AVOID_MARGIN` | `0.7` | Stop 0.7m from obstacle |
| `AVOID_BACKUP_SPD` | `0.3` | Reverse speed when too close |

> ⚠️ `PRX1_TYPE = 2` (not 5) is critical. Type 5 = direct-wired lidar. Type 2 = MAVLink messages from RPi.

### Spring-Center Throttle Transmitter

| Parameter | Value | Notes |
|---|---|---|
| `PILOT_THR_BHV` | `7` | Spring-center throttle support (bits 0+1+2) |
| `RC3_TRIM` | `1500` | Throttle center position |

### Flight Modes (set your mode switch)

Ensure one switch position is set to **LOITER (mode 5)**. Example config:

| Switch Position | Mode |
|---|---|
| 1 | ALTHOLD (2) |
| 2 | STABILIZE (0) |
| 3 | STABILIZE (0) |
| **4** | **LOITER (5) ← use this for flying** |
| 5 | STABILIZE (0) |
| 6 | LAND (9) |

---

## 🖥️ Raspberry Pi Setup

### Step 1 — Enable UART on GPIO 14/15

The GPIO UART (`/dev/ttyAMA0`) is used by Bluetooth by default. We need it for FC communication.

Add to `/boot/firmware/config.txt` (or `/boot/config.txt` on older OS):
```
dtoverlay=disable-bt
enable_uart=1
```

Remove `console=serial0,115200` from `/boot/firmware/cmdline.txt`.

Stop Bluetooth UART service:
```bash
sudo systemctl stop hciuart
sudo systemctl disable hciuart
sudo reboot
```

### Step 2 — Install SSH Server
```bash
sudo apt update && sudo apt install -y openssh-server
sudo systemctl enable ssh --now
```

### Step 3 — Install Python Dependencies
```bash
sudo pip3 install --break-system-packages pymavlink rplidar-roboticia pyserial
```

### Step 4 — Install lidar_bridge as a systemd service

Create `/etc/systemd/system/lidar_bridge.service`:
```ini
[Unit]
Description=RPLidar A1M8 to MAVLink Bridge
After=local-fs.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=YOUR_USERNAME
WorkingDirectory=/path/to/Position Hold
ExecStartPre=/bin/sleep 15
ExecStart=/usr/bin/python3 "/path/to/Position Hold/lidar_bridge.py"
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable lidar_bridge
sudo systemctl start lidar_bridge
```

### Step 5 — Verify everything works
```bash
# Check lidar bridge is running
sudo systemctl status lidar_bridge

# Watch live logs
journalctl -u lidar_bridge -f
```

You should see:
```
FC connected. System=1, Component=0
RPLidar health: Good
Streaming OBSTACLE_DISTANCE messages to ArduPilot...
Scan #50 | Valid sectors: 54/72 | Closest: 87 cm
```

Then verify in **Mission Planner → Ctrl+F → Proximity** — you should see a live radar of walls.

---

## 🔧 Lidar Mounting Angle Calibration

In `lidar_bridge.py`, find and set:
```python
LIDAR_FORWARD_ANGLE = 0   # Set this to the RPLidar angle that faces drone front
```

| Cable position on RPLidar motor | Set value |
|---|---|
| Cable faces **back** (most common) | `0` |
| Cable faces **right** | `90` |
| Cable faces **front** | `180` |
| Cable faces **left** | `270` |

**Verify:** Run the bridge, open Mission Planner → Proximity. Put a wall directly in front. It should appear at 12 o'clock on the radar.

---

## 📁 Project Files

| File | Description |
|---|---|
| `lidar_bridge.py` | Runs on boot as systemd service. Reads RPLidar A1M8 and sends `OBSTACLE_DISTANCE` MAVLink messages to ArduPilot at 10 Hz |
| `takeoff.py` | Autonomous takeoff script. Sets EKF origin, arms in GUIDED, climbs to 1.5m, switches to LOITER |
| `takeoff_avoidance_test.py` | Same as takeoff.py but uses SLIDE avoidance mode for indoor testing + live proximity monitor |
| `arm_disarm.py` | Interactive arm/disarm/status script |
| `requirements.txt` | Python dependencies |
| `setup_rpi.sh` | One-time RPi setup script (UART, packages, systemd service) |

---

## 🚁 Flight Procedures

### Pre-flight Checklist

- [ ] Cube Orange parameters set and FC rebooted
- [ ] BEC output set to 5.1–5.2V (not 5.0V)
- [ ] Both 5V GPIO pins (Pin 2 + Pin 4) connected to BEC
- [ ] RPLidar USB connected to RPi
- [ ] GPS2 port wired to RPi GPIO (TX→Pin10, RX→Pin8, GND→Pin6)
- [ ] lidar_bridge service running (`systemctl status lidar_bridge`)
- [ ] Proximity radar visible in Mission Planner (Ctrl+F → Proximity)
- [ ] MTF-01P optical flow data showing in Mission Planner Status tab
- [ ] RC transmitter ON, mode switch on **LOITER position**
- [ ] Throttle stick pushed **DOWN**

### Autonomous Takeoff (Normal Flight)
```bash
cd "/path/to/Position Hold"
python3 takeoff.py
```
After hover: pick up RC transmitter and fly. Drone stops 0.7m from walls automatically.

### Avoidance Test Flight
```bash
python3 takeoff_avoidance_test.py
```
Drone uses SLIDE mode (moves along walls instead of freezing). Watch live proximity in terminal. Press `Ctrl+C` when done — restores production STOP mode automatically.

### Arm / Disarm
```bash
python3 arm_disarm.py             # Interactive menu
python3 arm_disarm.py arm         # Arm directly
python3 arm_disarm.py disarm      # Disarm directly
python3 arm_disarm.py status      # Show arm status, mode, altitude
```

### Landing
1. Use throttle stick below center to descend in LOITER mode
2. Once on ground, hold throttle at zero for 2 seconds → auto-disarms
3. Or: `python3 arm_disarm.py disarm`

### SSH Access (headless operation)
```bash
# Find RPi IP first (on RPi terminal)
hostname -I

# Connect from laptop
ssh username@<RPi-IP>
```

---

## 🛡️ Obstacle Avoidance Behaviour

| Mode | `AVOID_BEHAVE` | Behaviour |
|---|---|---|
| **STOP** (production) | `1` | Drone freezes 0.7m from any obstacle. Cannot be overridden by RC stick. |
| **SLIDE** (test) | `0` | Drone slides along wall surface. Can move parallel to wall but not through it. |

`AVOID_BACKUP_SPD = 0.3 m/s` — drone slowly backs away if it drifts inside the margin.

> In **LOITER mode only** — obstacle avoidance is active. In STABILIZE/ALTHOLD, avoidance does not apply.

---

## 🔍 Troubleshooting

### Proximity not showing in Mission Planner
→ Ensure `PRX1_TYPE = 2` (not 5). Reboot FC after changing.

### Drone won't arm
- EKF not initialised → wait 30–45s after boot, hold drone level
- RC throttle not low → push throttle stick fully down before arming
- RC failsafe → turn transmitter ON before running takeoff script
- Check pre-arm messages in Mission Planner

### Drone arms but won't takeoff
- EKF timeout → run `takeoff.py` again (EKF needs ~30s after boot)
- Check optical flow data in Mission Planner Status tab (`opt_qua` field)

### Throttle not responding in flight
- Check mode is LOITER (not GUIDED/STABILIZE) in Mission Planner
- Ensure `PILOT_THR_BHV = 7` is set
- Ensure mode switch is on LOITER position

### Drone frozen in place, won't move with RC
- `AVOID_BEHAVE = 1` (STOP) and surrounded by walls within 0.7m
- Use `takeoff_avoidance_test.py` instead (SLIDE mode, 0.5m margin)
- Or fly in a larger space

### Under-voltage / PMIC reset on boot
- Raise BEC output to 5.1–5.2V
- Connect both 5V GPIO pins (Pin 2 AND Pin 4)
- Disconnect display/keyboard/mouse when flying headless
- Use thicker power wires (minimum 20 AWG)

### SSH connection refused
- Enable SSH: `sudo apt install openssh-server && sudo systemctl enable ssh --now`
- Check firewall: `sudo ufw allow ssh`
- iPhone hotspot blocks device-to-device traffic → use router WiFi or enable "Maximize Compatibility"

---

## 📐 System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Raspberry Pi                         │
│                                                         │
│  RPLidar A1M8 ──USB──► lidar_bridge.py (systemd)       │
│  /dev/ttyUSB0              │                            │
│                            │ OBSTACLE_DISTANCE          │
│  takeoff.py ───────────────┤ MAVLink messages           │
│  arm_disarm.py ────────────┤ at 10 Hz                   │
│                            │                            │
│                       /dev/ttyAMA0 (GPIO 14/15)         │
└────────────────────────────┼────────────────────────────┘
                             │ UART @ 921600 baud
                             │ (GPS2 port / SERIAL4)
┌────────────────────────────┼────────────────────────────┐
│              Cube Orange   │                            │
│                            ▼                            │
│  SERIAL4 ◄─── MAVLink ─── RPi                          │
│  SERIAL2 ◄─── MAVLink ─── MTF-01P (optical flow)       │
│                                                         │
│  EKF3: position from optical flow                       │
│  PRX: proximity from OBSTACLE_DISTANCE messages         │
│  AVOID: stops/slides drone near walls                   │
└─────────────────────────────────────────────────────────┘
```

---

## 📝 License

MIT License — free to use, modify, and distribute.
