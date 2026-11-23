#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# Drone Auto-Start & Field Hotspot Network Setup (Raspberry Pi 5)
#
# Run ONCE as:
#   HOTSPOT_PASS="<your-wpa2-password>" sudo -E bash scripts/setup_autostart.sh
#
# Optional environment overrides:
#   DRONE_USER      Linux user to run the services (default: $SUDO_USER or $USER)
#   DRONE_VENV      Path to Python venv (default: <repo>/.venv)
#   HOTSPOT_SSID    Wi-Fi AP SSID (default: DroneNet)
#   HOTSPOT_PASS    Wi-Fi AP WPA2 passphrase (REQUIRED; min 8 chars)
#   HOTSPOT_IP      Static AP IPv4/CIDR (default: 10.42.0.1/24)
#   DRONE_HOSTNAME  mDNS hostname (default: drone -> http://drone.local:8765)
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USR="${DRONE_USER:-${SUDO_USER:-$USER}}"
VENV="${DRONE_VENV:-${REPO_DIR}/.venv}"
HOTSPOT_SSID="${HOTSPOT_SSID:-DroneNet}"
HOTSPOT_PASS="${HOTSPOT_PASS:-}"
HOTSPOT_CIDR="${HOTSPOT_IP:-10.42.0.1/24}"
HOTSPOT_ADDR="${HOTSPOT_CIDR%%/*}"
DRONE_HOSTNAME="${DRONE_HOSTNAME:-drone}"

if [[ -z "${HOTSPOT_PASS}" || ${#HOTSPOT_PASS} -lt 8 ]]; then
    echo "ERROR: Please provide HOTSPOT_PASS (min 8 characters), e.g.:"
    echo "  HOTSPOT_PASS=\"ChangeMe123\" sudo -E bash scripts/setup_autostart.sh"
    exit 1
fi

echo ""
echo "==================================================="
echo "  Drone Auto-Start and Network Setup"
echo "  Repo : ${REPO_DIR}"
echo "  User : ${USR}"
echo "==================================================="

# ── 1. SYSTEMD SERVICES ───────────────────────────────────────────────────────
echo ""
echo "[1/4] Installing systemd services..."

cat > /etc/systemd/system/start_drone.service << EOF
[Unit]
Description=Drone Sensor Bridge (src/start_drone.py)
After=multi-user.target
StartLimitIntervalSec=60
StartLimitBurst=5

[Service]
Type=simple
User=${USR}
WorkingDirectory=${REPO_DIR}
ExecStart=${VENV}/bin/python3 ${REPO_DIR}/src/start_drone.py
Restart=on-failure
RestartSec=5s
StandardOutput=journal
StandardError=journal
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

cat > /etc/systemd/system/drone_dashboard.service << EOF
[Unit]
Description=Drone 3D GCS Dashboard (src/dashboard/bridge.py)
After=start_drone.service network.target
Wants=start_drone.service

[Service]
Type=simple
User=${USR}
WorkingDirectory=${REPO_DIR}
ExecStart=${VENV}/bin/python3 ${REPO_DIR}/src/dashboard/bridge.py
Restart=on-failure
RestartSec=8s
StandardOutput=journal
StandardError=journal
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable start_drone.service drone_dashboard.service
echo "   OK: start_drone.service enabled"
echo "   OK: drone_dashboard.service enabled"

# ── 2. WIFI HOTSPOT ───────────────────────────────────────────────────────────
echo ""
echo "[2/4] Setting up Wi-Fi hotspot '${HOTSPOT_SSID}'..."

nmcli con delete "DroneHotspot" 2>/dev/null || true
nmcli con add type wifi ifname wlan0 con-name "DroneHotspot" autoconnect no ssid "${HOTSPOT_SSID}"
nmcli con modify "DroneHotspot" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    802-11-wireless.channel 6 \
    ipv4.method shared \
    ipv4.addresses "${HOTSPOT_CIDR}" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "${HOTSPOT_PASS}"

echo "   OK: Hotspot '${HOTSPOT_SSID}' created"
echo "   OK: Pi fixed IP when hotspot active: ${HOTSPOT_ADDR}"
echo "   OK: Dashboard URL: http://${HOTSPOT_ADDR}:8765"

# ── 3. TOGGLE COMMANDS ────────────────────────────────────────────────────────
echo ""
echo "[3/4] Installing drone-hotspot-on / drone-hotspot-off commands..."

cat > /usr/local/bin/drone-hotspot-on << EOF
#!/bin/bash
echo "Activating ${HOTSPOT_SSID} hotspot..."
nmcli con up DroneHotspot
echo "Connect to Wi-Fi: ${HOTSPOT_SSID}"
echo "Dashboard: http://${HOTSPOT_ADDR}:8765"
EOF

cat > /usr/local/bin/drone-hotspot-off << 'EOF'
#!/bin/bash
echo "Deactivating DroneHotspot..."
nmcli con down DroneHotspot 2>/dev/null || true
echo "Reconnect to your Wi-Fi network manually if needed."
EOF

chmod +x /usr/local/bin/drone-hotspot-on /usr/local/bin/drone-hotspot-off
echo "   OK: drone-hotspot-on installed"
echo "   OK: drone-hotspot-off installed"

# ── 4. mDNS HOSTNAME ──────────────────────────────────────────────────────────
echo ""
echo "[4/4] Setting mDNS hostname to '${DRONE_HOSTNAME}'..."
hostnamectl set-hostname "${DRONE_HOSTNAME}"
echo "   OK: hostname = ${DRONE_HOSTNAME} -> http://${DRONE_HOSTNAME}.local:8765"

echo ""
echo "==================================================="
echo "  DONE"
echo "==================================================="
echo ""
echo "  FIELD USE (no external Wi-Fi):"
echo "    sudo drone-hotspot-on"
echo "    Open: http://${HOTSPOT_ADDR}:8765"
echo ""
echo "  LAB USE (shared Wi-Fi):"
echo "    Open: http://${DRONE_HOSTNAME}.local:8765"
echo ""
echo "  Logs:"
echo "    journalctl -fu start_drone"
echo "    journalctl -fu drone_dashboard"
echo ""
echo "  Reboot to activate auto-start, or start now:"
echo "    sudo systemctl start start_drone drone_dashboard"
echo ""
