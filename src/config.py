"""
config.py — Central configuration for the Navigation_ROSLess drone system.

ALL hardware paths, baud rates, timing constants, and tuning knobs live here.
Serial ports, baud rates, and network ports can be overridden via environment
variables (e.g., for SITL simulation or different Raspberry Pi UART mappings)
without modifying source code.
"""

import os

# ─────────────────────────────────────────────────────────────────────────────
# SERIAL DEVICE PATHS  (override via environment variables if needed)
# ─────────────────────────────────────────────────────────────────────────────

# Pi ↔ Flight Controller (MAVLink 2, 921600 baud)
# For ArduPilot SITL, set FC_SERIAL_PORT=tcp:127.0.0.1:5762 or udp:127.0.0.1:14550
FC_SERIAL_PORT   = os.environ.get("FC_SERIAL_PORT", "/dev/ttyAMA4")
FC_BAUD_RATE     = int(os.environ.get("FC_BAUD_RATE", "921600"))

# Downward MTF-01P — Optical Flow + Rangefinder (MicoLink / mav_apm, 115200 baud)
MTF_DOWN_PORT    = os.environ.get("MTF_DOWN_PORT", "/dev/ttyAMA0")
MTF_DOWN_BAUD    = int(os.environ.get("MTF_DOWN_BAUD", "115200"))

# Upward MTF-01P — Rangefinder only (MicoLink, 115200 baud)
MTF_UP_PORT      = os.environ.get("MTF_UP_PORT", "/dev/ttyAMA2")
MTF_UP_BAUD      = int(os.environ.get("MTF_UP_BAUD", "115200"))

# RPLidar A1 — 360° 2D LiDAR (USB, udev symlinked to /dev/rplidar)
RPLIDAR_PORT     = os.environ.get("RPLIDAR_PORT", "/dev/rplidar")
RPLIDAR_BAUD     = int(os.environ.get("RPLIDAR_BAUD", "115200"))   # rplidar-roboticia default

# ─────────────────────────────────────────────────────────────────────────────
# MAVLINK SYSTEM IDENTITY
# ─────────────────────────────────────────────────────────────────────────────

# The companion computer's MAVLink identity (sent in HEARTBEAT)
GCS_SYSTEM_ID    = 255             # Standard GCS ID — FC responds to commands from this
GCS_COMPONENT_ID = 0               # MAV_COMP_ID_ALL

# Flight Controller expected identity (verified on first heartbeat)
FC_SYSTEM_ID     = 1
FC_COMPONENT_ID  = 1               # MAV_COMP_ID_AUTOPILOT1

# ─────────────────────────────────────────────────────────────────────────────
# MAVLINK STREAM RATES
# ─────────────────────────────────────────────────────────────────────────────

FLOW_HZ          = 50              # OPTICAL_FLOW_RAD publish rate (Hz)  ← ArduPilot EKF3 needs ≥30 Hz
DISTANCE_HZ      = 30              # DISTANCE_SENSOR publish rate (Hz)
OBSTACLE_HZ      = 10             # OBSTACLE_DISTANCE publish rate (Hz)
HEARTBEAT_HZ     = 1              # Companion heartbeat to FC (Hz)

# ─────────────────────────────────────────────────────────────────────────────
# MICOLINK PROTOCOL CONSTANTS  (MTF-01P default output)
# ─────────────────────────────────────────────────────────────────────────────

MICOLINK_HEADER  = 0xEF           # Fixed start-of-frame byte
MICOLINK_MSG_ID  = 0x51           # Range + Flow combined message ID
MICOLINK_PAYLOAD_LEN = 20         # Bytes in the 0x51 payload
# Full frame = 1 head + 1 dev_id + 1 sys_id + 1 msg_id + 1 seq + 1 len
#             + 20 payload + 1 checksum = 27 bytes
# (The C struct has 'status' and 'payload_cnt' fields but those are NOT wire bytes)
MICOLINK_FRAME_LEN = 27

# ─────────────────────────────────────────────────────────────────────────────
# SENSOR PHYSICAL LIMITS
# ─────────────────────────────────────────────────────────────────────────────

# Rangefinder valid range (mm).  Values outside this are discarded.
RNGFND_MIN_MM    = 100            # 10 cm minimum valid reading
RNGFND_MAX_MM    = 8000           # 8 m maximum valid reading

# Optical flow quality threshold (0–255).  Below this, flow data is not forwarded.
FLOW_QUALITY_MIN = 50

# MTF-01P sensor IDs in the MAVLink DISTANCE_SENSOR message
DIST_SENSOR_ID_DOWN = 0           # Downward rangefinder (MAV_SENSOR_ROTATION_PITCH_270 = 25)
DIST_SENSOR_ID_UP   = 1           # Upward rangefinder   (MAV_SENSOR_ROTATION_PITCH_90  = 24)

# ─────────────────────────────────────────────────────────────────────────────
# RPLIDAR OBSTACLE MAP
# ─────────────────────────────────────────────────────────────────────────────

OBSTACLE_SECTORS       = 72       # Number of sectors in OBSTACLE_DISTANCE (72 × 5° = 360°)
OBSTACLE_SECTOR_DEG    = 5        # Degrees per sector
OBSTACLE_MIN_CM        = 20       # Minimum reported distance (cm)
OBSTACLE_MAX_CM        = 800      # Maximum reported distance (cm)  — RPLidar A1 effective range

# ─────────────────────────────────────────────────────────────────────────────
# WATCHDOG / HEALTH TIMEOUTS
# ─────────────────────────────────────────────────────────────────────────────

SENSOR_STALE_S         = 2.0     # Seconds before a sensor reading is considered stale
FC_HEARTBEAT_TIMEOUT_S = 5.0     # Seconds to wait before declaring FC connection lost

# ─────────────────────────────────────────────────────────────────────────────
# NAVIGATION PARAMETERS  (used by auto_navigate.py)
# ─────────────────────────────────────────────────────────────────────────────

# Takeoff
TAKEOFF_ALT_M          = 1.0     # Target hover altitude (m)
TAKEOFF_TOLERANCE_M    = 0.10    # ±10 cm considered "at altitude"

# Ceiling guard — upward sensor must read more than this to allow upward motion
CEILING_MIN_M          = 0.40    # 40 cm clearance above drone required

# Wall-following
WALL_FOLLOW_SPEED_MS   = 0.35   # Forward cruise speed (m/s)
WALL_TARGET_DIST_M     = 0.60   # Desired right-side wall distance (m)
WALL_P_GAIN            = 0.8    # Proportional gain for wall distance correction
FRONT_BRAKE_DIST_M     = 0.50   # Front obstacle distance that triggers turn
TURN_RATE_DEG_S        = 40.0   # Yaw turn rate during corner manoeuvre (deg/s)
TURN_90_DEG_S          = 90.0 / TURN_RATE_DEG_S   # Time to execute a 90° turn (s)

# Control loop timing
NAV_LOOP_HZ            = 10     # Navigation decision loop rate (Hz)
NAV_LOOP_S             = 1.0 / NAV_LOOP_HZ

# EKF health soak time before arming (seconds)
EKF_SOAK_S             = 8.0

# Velocity command repeat interval — ArduPilot times out velocity targets after ~3 s
# so we must re-send at least this often
VEL_CMD_REPEAT_S       = 0.5

# ─────────────────────────────────────────────────────────────────────────────
# MAVLINK TCP BRIDGE
# ─────────────────────────────────────────────────────────────────────────────
# start_drone.py exposes a local TCP MAVLink server so auto_navigate.py
# (and any other GCS tool) can connect without needing the serial port.
# auto_navigate.py connects to:  tcp:127.0.0.1:<FC_BRIDGE_PORT>

FC_BRIDGE_HOST         = os.environ.get("FC_BRIDGE_HOST", "127.0.0.1")
FC_BRIDGE_PORT         = int(os.environ.get("FC_BRIDGE_PORT", "5760"))  # Local MAVLink TCP bridge port
