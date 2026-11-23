#!/usr/bin/env bash
# setup_rpi_position_hold.sh — One-time Raspberry Pi setup for the Cube Orange
# Position-Hold + Proximity Avoidance companion stack.
#
# Run ONCE on the Raspberry Pi as root:
#   sudo bash scripts/setup_rpi_position_hold.sh
#
# Optional environment overrides:
#   PI_USER     Linux user (default: $SUDO_USER or pi)
#   DRONE_DIR   Install directory (default: /home/$PI_USER/drone)

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${REPO_DIR}/src/position_hold"
SERVICE_NAME="lidar_bridge"
PI_USER="${PI_USER:-${SUDO_USER:-pi}}"
DRONE_DIR="${DRONE_DIR:-/home/${PI_USER}/drone}"

echo ""
echo "=========================================="
echo " Drone Companion Computer Setup (Position-Hold)"
echo " User : ${PI_USER}"
echo " Dest : ${DRONE_DIR}"
echo "=========================================="
echo ""

# ── 1. Enable hardware UART on GPIO 14/15 ──────────────────────────────────
echo "[1/6] Enabling hardware UART on GPIO 14/15..."

CONFIG_FILE="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/boot/config.txt"
fi

if ! grep -q "dtoverlay=disable-bt" "$CONFIG_FILE"; then
    echo "dtoverlay=disable-bt" >> "$CONFIG_FILE"
    echo "  Added: dtoverlay=disable-bt"
fi

if ! grep -q "enable_uart=1" "$CONFIG_FILE"; then
    echo "enable_uart=1" >> "$CONFIG_FILE"
    echo "  Added: enable_uart=1"
fi

echo "  Done."

# ── 2. Disable serial console ──────────────────────────────────────────────
echo "[2/6] Disabling serial console on /dev/ttyAMA0..."

CMDLINE_FILE="/boot/firmware/cmdline.txt"
if [ ! -f "$CMDLINE_FILE" ]; then
    CMDLINE_FILE="/boot/cmdline.txt"
fi

if grep -q "console=serial0" "$CMDLINE_FILE" || grep -q "console=ttyAMA0" "$CMDLINE_FILE"; then
    sed -i 's/console=serial0,[0-9]* //g' "$CMDLINE_FILE"
    sed -i 's/console=ttyAMA0,[0-9]* //g' "$CMDLINE_FILE"
    echo "  Removed serial console from cmdline.txt"
else
    echo "  Serial console was not set (already clean)"
fi

systemctl stop hciuart 2>/dev/null || true
systemctl disable hciuart 2>/dev/null || true
echo "  Disabled Bluetooth UART service."
echo "  Done."

# ── 3. Install Python dependencies ─────────────────────────────────────────
echo "[3/6] Installing Python packages..."

apt-get update -qq
apt-get install -y python3-pip python3-serial python3-dev

pip3 install --break-system-packages -r "${REPO_DIR}/requirements.txt"
echo "  Done."

# ── 4. Copy project files ──────────────────────────────────────────────────
echo "[4/6] Copying drone scripts to ${DRONE_DIR}..."

mkdir -p "$DRONE_DIR"
cp "$SRC_DIR/lidar_bridge.py"            "$DRONE_DIR/"
cp "$SRC_DIR/takeoff.py"                 "$DRONE_DIR/"
cp "$SRC_DIR/takeoff_avoidance_test.py"  "$DRONE_DIR/"
cp "$SRC_DIR/arm_disarm.py"              "$DRONE_DIR/"
cp "$REPO_DIR/requirements.txt"          "$DRONE_DIR/"
chown -R "$PI_USER:$PI_USER" "$DRONE_DIR"
chmod +x "$DRONE_DIR"/*.py

echo "  Files copied to ${DRONE_DIR}"
echo "  Done."

# ── 5. Install systemd service ─────────────────────────────────────────────
echo "[5/6] Installing ${SERVICE_NAME} systemd service..."

cat > /etc/systemd/system/${SERVICE_NAME}.service << EOF
[Unit]
Description=RPLidar A1M8 to MAVLink Bridge
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=${PI_USER}
WorkingDirectory=${DRONE_DIR}
ExecStart=/usr/bin/python3 ${DRONE_DIR}/lidar_bridge.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}.service"
echo "  Service installed and enabled."
echo "  Done."

# ── 6. Add user to dialout group (for serial/USB access) ───────────────────
echo "[6/6] Adding '${PI_USER}' to dialout group..."
usermod -a -G dialout "$PI_USER"
echo "  Done."

echo ""
echo "=========================================="
echo " Setup complete!"
echo "=========================================="
echo ""
echo " NEXT STEPS:"
echo "  1. REBOOT the Raspberry Pi:  sudo reboot"
echo "  2. After reboot, lidar_bridge.py starts automatically."
echo "  3. Verify it's running:      sudo systemctl status lidar_bridge"
echo "  4. View live logs:           journalctl -u lidar_bridge -f"
echo "  5. Test lidar in Mission Planner: Ctrl+F -> Proximity"
echo "  6. To trigger takeoff:       python3 ${DRONE_DIR}/takeoff.py"
echo ""
