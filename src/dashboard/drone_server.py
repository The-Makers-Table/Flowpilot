#!/usr/bin/env python3
"""
drone_server.py — Minimal Drone Control Dashboard
==================================================
Self-contained FastAPI + WebSocket server with embedded HTML UI.
Serves the control dashboard at http://<PI_IP>:8765 (default port 8765).

Controls:
  - TAKEOFF & AUTO-LOITER → Climbs in GUIDED to target altitude, then automatically
                            switches to LOITER mode to engage MTF-01 optical flow hold.
  - LOITER / HOLD         → Manually switch to LOITER mode at any time.
  - ARM                   → Arms motors in GUIDED mode.
  - DISARM                → Force-disarms immediately (emergency override).
  - LAND                  → Commands LAND mode.

Hardware: SpeedyBee F405 Mini · MTF-01P Optical Flow · RPi 5
No GPS · No RC Controller · No Compass
"""

import asyncio
import json
import os
import threading
import time
import logging
import socket
from typing import Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────────────────────
FC_PORT       = os.environ.get("FC_SERIAL_PORT", "/dev/ttyAMA4")  # RPi 5 UART4 — GPIO 12 (TX) / 13 (RX)
FC_BAUD       = int(os.environ.get("FC_BAUD_RATE", "921600"))     # Must match SERIAL2_BAUD on SpeedyBee
SERVER_HOST   = os.environ.get("DASHBOARD_HOST", "0.0.0.0")
SERVER_PORT   = int(os.environ.get("DASHBOARD_PORT", "8765"))
HOTSPOT_IP    = os.environ.get("DRONE_HOTSPOT_IP", "<HOTSPOT_IP>")
ALT_TOLERANCE = 0.10            # Switch to LOITER when within 10cm of target
ALT_TIMEOUT   = 30.0            # Max seconds to wait for altitude before giving up
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

app = FastAPI(title="Drone Dashboard")

# ─── Shared State ─────────────────────────────────────────────────────────────
state = {
    "connected": False,
    "armed":     False,
    "mode":      "UNKNOWN",
    "alt":       0.0,
    "ekf_ok":    False,
    "ekf_flags": {"att": False, "vel": False, "pos": False, "hgt": False},
    "target_alt": 1.0,
}
mav_conn   = None
state_lock = threading.Lock()
clients: Set[WebSocket] = set()

# Climb monitor thread handle / cancel flag
climb_thread_cancel = threading.Event()


# ─── Helper: Broadcast message to WebSockets ──────────────────────────────────
async def send_to_clients(payload_dict: dict):
    if not clients:
        return
    text = json.dumps(payload_dict)
    dead = set()
    for ws in list(clients):
        try:
            await ws.send_text(text)
        except Exception:
            dead.add(ws)
    clients.difference_update(dead)


# ─── MAVLink Background Thread ────────────────────────────────────────────────
def mavlink_loop():
    global mav_conn
    while True:
        try:
            log.info(f"MAVLink: Connecting to {FC_PORT} @ {FC_BAUD}...")
            conn = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
            conn.wait_heartbeat(timeout=10)
            conn.mav.request_data_stream_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
            )
            mav_conn = conn
            with state_lock:
                state["connected"] = True
            log.info(f"MAVLink: Connected! System={conn.target_system}")

            while True:
                msg = conn.recv_match(blocking=True, timeout=1)
                if msg is None:
                    try:
                        conn.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0, 0, 0
                        )
                    except Exception:
                        break
                    continue

                mt = msg.get_type()

                if mt == 'HEARTBEAT' and msg.get_srcSystem() == conn.target_system:
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mode_map = {v: k for k, v in conn.mode_mapping().items()}
                    mode = mode_map.get(msg.custom_mode, str(msg.custom_mode))
                    with state_lock:
                        state["armed"] = armed
                        state["mode"]  = mode

                elif mt == 'GLOBAL_POSITION_INT':
                    with state_lock:
                        state["alt"] = round(msg.relative_alt / 1000.0, 2)

                elif mt == 'RANGEFINDER':
                    # Optical flow lidar distance in metres
                    with state_lock:
                        state["alt"] = round(msg.distance, 2)

                elif mt == 'EKF_STATUS_REPORT':
                    flags = msg.flags
                    att = bool(flags & (1 << 0))
                    vel = bool(flags & (1 << 1))
                    pos = bool(flags & (1 << 3))
                    hgt = bool(flags & (1 << 6))
                    with state_lock:
                        state["ekf_ok"]    = att and vel
                        state["ekf_flags"] = {
                            "att": att, "vel": vel,
                            "pos": pos, "hgt": hgt,
                        }

        except Exception as exc:
            log.warning(f"MAVLink: Connection lost — {exc}. Retrying in 3s...")
            mav_conn = None
            with state_lock:
                state["connected"] = False
                state["armed"]     = False
                state["ekf_ok"]    = False
            time.sleep(3)


# ─── Telemetry Broadcast Loop ─────────────────────────────────────────────────
async def broadcast_loop():
    while True:
        await asyncio.sleep(0.5)
        if not clients:
            continue
        with state_lock:
            snapshot = dict(state)
        snapshot["type"] = "telemetry"
        await send_to_clients(snapshot)


# ─── Background Climb Monitor: Auto-switch to LOITER ──────────────────────────
def monitor_climb_and_switch_loiter(conn, target_alt: float):
    """
    Watches altitude while climbing in GUIDED.
    Once within ALT_TOLERANCE of target_alt, commands LOITER mode.
    """
    log.info(f"ClimbMonitor: Started. Climbing to {target_alt}m... will switch to LOITER at target.")
    deadline = time.time() + ALT_TIMEOUT

    while time.time() < deadline:
        if climb_thread_cancel.is_set():
            log.info("ClimbMonitor: Cancelled (disarm or land commanded).")
            return

        with state_lock:
            current_alt = state["alt"]
            armed       = state["armed"]
            current_mode= state["mode"]

        if not armed:
            log.info("ClimbMonitor: Disarmed — stopping climb monitor.")
            return

        if current_mode == 'LAND':
            log.info("ClimbMonitor: LAND commanded — stopping climb monitor.")
            return

        # Check if target altitude reached
        if current_alt >= (target_alt - ALT_TOLERANCE):
            log.info(f"ClimbMonitor: ✅ Target altitude reached ({current_alt:.2f}m >= {target_alt:.2f}m).")
            log.info("ClimbMonitor: Switching to LOITER mode for Optical Flow position hold...")

            loiter_mode_id = conn.mode_mapping().get('LOITER')
            if loiter_mode_id is not None:
                conn.mav.set_mode_send(
                    conn.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    loiter_mode_id
                )
                log.info("ClimbMonitor: Sent LOITER mode command!")
            else:
                log.error("ClimbMonitor: LOITER mode ID not found in mapping!")
            return

        time.sleep(0.1)

    log.warning(f"ClimbMonitor: Timed out after {ALT_TIMEOUT}s without reaching {target_alt}m.")


# ─── Command Executor ─────────────────────────────────────────────────────────
def execute_command(cmd: dict) -> dict:
    global mav_conn
    if mav_conn is None:
        return {"ok": False, "msg": "FC not connected — check wiring"}

    action = cmd.get("action", "")
    conn   = mav_conn

    try:
        # ── ARM ───────────────────────────────────────────────────────────────
        if action == "arm":
            log.info("CMD: ARM")
            climb_thread_cancel.set()
            conn.mav.set_gps_global_origin_send(conn.target_system, 0, 0, 0)
            time.sleep(0.3)
            mode_id = conn.mode_mapping().get('GUIDED')
            conn.mav.set_mode_send(
                conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            time.sleep(0.5)
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            return {"ok": True, "msg": "ARM sent — motors spinning up"}

        # ── DISARM ────────────────────────────────────────────────────────────
        elif action == "disarm":
            log.info("CMD: DISARM (force)")
            climb_thread_cancel.set()
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,      # Disarm
                21196,  # Force-disarm override (works mid-flight)
                0, 0, 0, 0, 0
            )
            return {"ok": True, "msg": "DISARM sent — motors stopped"}

        # ── TAKEOFF + AUTO LOITER ──────────────────────────────────────────────
        elif action == "takeoff":
            alt = float(cmd.get("alt", 1.0))
            alt = max(0.3, min(3.0, alt))
            log.info(f"CMD: TAKEOFF to {alt}m with Auto-Loiter")

            with state_lock:
                state["target_alt"] = alt

            climb_thread_cancel.clear()

            # 1. Set EKF local origin
            conn.mav.set_gps_global_origin_send(conn.target_system, 0, 0, 0)
            time.sleep(0.3)

            # 2. Switch to GUIDED
            mode_id = conn.mode_mapping().get('GUIDED')
            conn.mav.set_mode_send(
                conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            time.sleep(0.5)

            # 3. Arm
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            time.sleep(1.2)

            # 4. Takeoff
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0,
                float('nan'),
                0, 0,
                alt
            )

            # 5. Launch background monitor thread to switch to LOITER at target height
            t = threading.Thread(
                target=monitor_climb_and_switch_loiter,
                args=(conn, alt),
                daemon=True
            )
            t.start()

            return {
                "ok": True,
                "msg": f"Climbing to {alt}m... Will auto-switch to LOITER at target altitude!"
            }

        # ── MANUAL LOITER ─────────────────────────────────────────────────────
        elif action == "loiter":
            log.info("CMD: Manual LOITER switch")
            mode_id = conn.mode_mapping().get('LOITER')
            if mode_id is not None:
                conn.mav.set_mode_send(
                    conn.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
                return {"ok": True, "msg": "LOITER mode commanded — optical flow hold active"}
            else:
                return {"ok": False, "msg": "LOITER mode not available"}

        # ── LAND ──────────────────────────────────────────────────────────────
        elif action == "land":
            log.info("CMD: LAND")
            climb_thread_cancel.set()
            mode_id = conn.mode_mapping().get('LAND')
            conn.mav.set_mode_send(
                conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            return {"ok": True, "msg": "LAND mode active — descending..."}

        else:
            return {"ok": False, "msg": f"Unknown action: {action}"}

    except Exception as exc:
        log.error(f"Command error: {exc}")
        return {"ok": False, "msg": f"Error: {exc}"}


# ─── WebSocket Endpoint ───────────────────────────────────────────────────────
@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    log.info(f"Browser connected ({len(clients)} total)")
    try:
        while True:
            raw    = await websocket.receive_text()
            cmd    = json.loads(raw)
            loop   = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, execute_command, cmd)
            result["type"] = "ack"
            await websocket.send_text(json.dumps(result))
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(websocket)
        log.info(f"Browser disconnected ({len(clients)} total)")


# ─── HTML Dashboard ───────────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Drone Control</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;900&display=swap" rel="stylesheet">
<style>
*, *::before, *::after { margin: 0; padding: 0; box-sizing: border-box; }

:root {
  --bg:         #080c18;
  --surface:    rgba(255,255,255,0.04);
  --border:     rgba(255,255,255,0.07);
  --border-2:   rgba(255,255,255,0.12);
  --text:       #e8eaf6;
  --text-dim:   rgba(232,234,246,0.45);
  --accent:     #4fc3f7;
  --green:      #69f0ae;
  --red:        #ff5252;
  --orange:     #ffb74d;
}

html, body {
  height: 100%;
  background: var(--bg);
  color: var(--text);
  font-family: 'Inter', -apple-system, sans-serif;
  -webkit-font-smoothing: antialiased;
}

body {
  min-height: 100dvh;
  max-width: 480px;
  margin: 0 auto;
  padding: 24px 20px 48px;
  background-image:
    radial-gradient(ellipse 60% 40% at 15% 10%, rgba(79,195,247,0.08) 0%, transparent 60%),
    radial-gradient(ellipse 50% 40% at 85% 90%, rgba(105,240,174,0.05) 0%, transparent 60%);
}

.header {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  margin-bottom: 24px;
}
.logo {
  display: flex;
  align-items: center;
  gap: 10px;
}
.logo-icon {
  width: 38px; height: 38px;
  background: linear-gradient(135deg, rgba(79,195,247,0.25), rgba(79,195,247,0.08));
  border: 1px solid rgba(79,195,247,0.3);
  border-radius: 10px;
  display: flex; align-items: center; justify-content: center;
  font-size: 1.1rem;
}
.logo-text h1 { font-size: 1.1rem; font-weight: 700; letter-spacing: -0.02em; }
.logo-text p  { font-size: 0.62rem; color: var(--text-dim); margin-top: 2px; letter-spacing: 0.05em; }

.conn-pill {
  display: flex; align-items: center; gap: 7px;
  padding: 7px 13px;
  background: var(--surface);
  border: 1px solid var(--border-2);
  border-radius: 20px;
  font-size: 0.72rem; font-weight: 500; color: var(--text-dim);
  white-space: nowrap;
  transition: all 0.3s;
}
.conn-dot {
  width: 7px; height: 7px; border-radius: 50%;
  background: var(--red); flex-shrink: 0;
  transition: background 0.3s;
}
.conn-dot.live {
  background: var(--green);
  box-shadow: 0 0 6px var(--green);
  animation: blink 2.5s infinite;
}
@keyframes blink { 0%,100%{opacity:1} 50%{opacity:0.35} }

/* Telemetry Grid */
.telem-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
  margin-bottom: 16px;
}
.tcard {
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 18px;
  padding: 16px;
}
.tcard.span2 { grid-column: 1 / -1; }
.tcard-label {
  font-size: 0.6rem; font-weight: 600;
  text-transform: uppercase; letter-spacing: 0.12em;
  color: var(--text-dim); margin-bottom: 8px;
}
.tcard-value {
  font-size: 1.65rem; font-weight: 800;
  letter-spacing: -0.03em; line-height: 1;
}
.tcard-unit { font-size: 0.85rem; font-weight: 400; color: var(--text-dim); margin-left: 3px; }
.mode-text  { font-size: 1.0rem; font-weight: 700; letter-spacing: 0.05em; color: var(--accent); }

/* Badges */
.badge {
  display: inline-flex; align-items: center; gap: 5px;
  padding: 5px 12px;
  border-radius: 20px;
  font-size: 0.68rem; font-weight: 700;
  text-transform: uppercase; letter-spacing: 0.06em;
  border: 1px solid transparent;
}
.badge-armed    { background: rgba(255,82,82,0.12);    color: var(--red);    border-color: rgba(255,82,82,0.3); }
.badge-disarmed { background: rgba(105,240,174,0.1);   color: var(--green);  border-color: rgba(105,240,174,0.25); }
.badge-ekfok    { background: rgba(105,240,174,0.1);   color: var(--green);  border-color: rgba(105,240,174,0.25); }
.badge-ekfbad   { background: rgba(255,183,77,0.1);    color: var(--orange); border-color: rgba(255,183,77,0.3); }

.ekf-row { font-size: 0.72rem; color: var(--text-dim); margin-top: 4px; line-height: 1.7; }

/* Altitude Setter */
.alt-card {
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 18px;
  padding: 20px;
  margin-bottom: 16px;
}
.alt-card-title {
  font-size: 0.62rem; font-weight: 600;
  text-transform: uppercase; letter-spacing: 0.12em;
  color: var(--text-dim); margin-bottom: 18px;
}
.alt-row { display: flex; align-items: center; gap: 20px; }
.alt-num {
  font-size: 2.8rem; font-weight: 900;
  color: var(--accent);
  letter-spacing: -0.04em; line-height: 1;
  min-width: 76px; text-align: right;
  font-variant-numeric: tabular-nums;
  transition: color 0.2s;
}
.alt-slider-wrap { flex: 1; }

input[type="range"] {
  width: 100%; -webkit-appearance: none;
  height: 5px; background: rgba(255,255,255,0.1);
  border-radius: 5px; outline: none; cursor: pointer;
}
input[type="range"]::-webkit-slider-thumb {
  -webkit-appearance: none;
  width: 24px; height: 24px; border-radius: 50%;
  background: var(--accent);
  box-shadow: 0 0 0 3px rgba(79,195,247,0.2), 0 0 14px rgba(79,195,247,0.45);
  cursor: pointer; transition: box-shadow 0.2s;
}
input[type="range"]:active::-webkit-slider-thumb {
  box-shadow: 0 0 0 7px rgba(79,195,247,0.15), 0 0 20px rgba(79,195,247,0.6);
}
input[type="range"]:disabled { opacity: 0.3; cursor: not-allowed; }
input[type="range"]:disabled::-webkit-slider-thumb { cursor: not-allowed; }

.alt-range-labels {
  display: flex; justify-content: space-between;
  margin-top: 8px;
  font-size: 0.6rem; color: var(--text-dim); letter-spacing: 0.05em;
}
.alt-note {
  margin-top: 14px;
  padding-top: 12px;
  border-top: 1px solid var(--border);
  font-size: 0.68rem; color: var(--text-dim); line-height: 1.5;
}

/* Controls */
.controls { display: flex; flex-direction: column; gap: 10px; }

.btn {
  width: 100%; padding: 18px 24px;
  border: none; border-radius: 16px;
  font-family: 'Inter', sans-serif;
  font-size: 0.95rem; font-weight: 700;
  letter-spacing: 0.03em;
  cursor: pointer;
  transition: all 0.15s ease;
  position: relative; overflow: hidden;
  display: flex; align-items: center; justify-content: center; gap: 8px;
  -webkit-tap-highlight-color: transparent;
}
.btn:active:not(:disabled) { transform: scale(0.97); }
.btn:disabled { opacity: 0.3; cursor: not-allowed; transform: none !important; }

.btn-takeoff {
  background: linear-gradient(135deg, #4fc3f7 0%, #0277bd 100%);
  color: #fff; font-size: 1.05rem; padding: 20px 24px;
  box-shadow: 0 4px 24px rgba(79,195,247,0.28), 0 1px 4px rgba(0,0,0,0.4);
}
.btn-takeoff:hover:not(:disabled) {
  box-shadow: 0 8px 36px rgba(79,195,247,0.44), 0 1px 4px rgba(0,0,0,0.4);
  transform: translateY(-1px);
}

.btn-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }

.btn-loiter {
  background: rgba(79,195,247,0.12);
  border: 1.5px solid rgba(79,195,247,0.4);
  color: var(--accent);
}
.btn-loiter:hover:not(:disabled) { background: rgba(79,195,247,0.22); border-color: rgba(79,195,247,0.65); }

.btn-land {
  background: linear-gradient(135deg, #ff5252 0%, #b71c1c 100%);
  color: #fff; font-size: 1.05rem; padding: 18px 24px;
  box-shadow: 0 4px 24px rgba(255,82,82,0.28), 0 1px 4px rgba(0,0,0,0.4);
}
.btn-land:hover:not(:disabled) {
  box-shadow: 0 8px 36px rgba(255,82,82,0.44), 0 1px 4px rgba(0,0,0,0.4);
  transform: translateY(-1px);
}

.btn-arm {
  background: rgba(255,183,77,0.1);
  border: 1.5px solid rgba(255,183,77,0.4);
  color: var(--orange);
}
.btn-arm:hover:not(:disabled) { background: rgba(255,183,77,0.18); border-color: rgba(255,183,77,0.65); }

.btn-disarm {
  background: rgba(105,240,174,0.08);
  border: 1.5px solid rgba(105,240,174,0.35);
  color: var(--green);
}
.btn-disarm:hover:not(:disabled) { background: rgba(105,240,174,0.16); border-color: rgba(105,240,174,0.55); }

/* Toast */
.toast {
  position: fixed; bottom: 28px; left: 50%;
  transform: translateX(-50%) translateY(90px);
  padding: 12px 22px;
  background: rgba(10,14,26,0.96);
  backdrop-filter: blur(20px);
  border: 1px solid rgba(255,255,255,0.1);
  border-radius: 14px;
  font-size: 0.82rem; font-weight: 500;
  transition: transform 0.35s cubic-bezier(0.34,1.56,0.64,1);
  z-index: 999; pointer-events: none;
  max-width: calc(100vw - 40px);
  text-align: center;
}
.toast.visible { transform: translateX(-50%) translateY(0); }
.toast.ok  { border-color: rgba(105,240,174,0.4); color: var(--green); }
.toast.err { border-color: rgba(255,82,82,0.4);   color: var(--red);   }
</style>
</head>
<body>

<header class="header">
  <div class="logo">
    <div class="logo-icon">⬡</div>
    <div class="logo-text">
      <h1>Drone Control</h1>
      <p>SPEEDYBEE F405 &middot; AUTO-LOITER HOLD</p>
    </div>
  </div>
  <div class="conn-pill">
    <span class="conn-dot" id="conn-dot"></span>
    <span id="conn-text">Connecting...</span>
  </div>
</header>

<div class="telem-grid">
  <div class="tcard">
    <div class="tcard-label">Armed State</div>
    <div id="armed-disp"><span class="badge badge-disarmed">&#x25CF; Disarmed</span></div>
  </div>
  <div class="tcard">
    <div class="tcard-label">Flight Mode</div>
    <div class="mode-text" id="mode-disp">&#8212;</div>
  </div>
  <div class="tcard">
    <div class="tcard-label">Altitude (AGL)</div>
    <div class="tcard-value" id="alt-disp">0.00<span class="tcard-unit">m</span></div>
  </div>
  <div class="tcard">
    <div class="tcard-label">EKF Health</div>
    <div id="ekf-badge-disp"><span class="badge badge-ekfbad">&#x23F3; Waiting</span></div>
  </div>
  <div class="tcard span2">
    <div class="tcard-label">EKF Sensors</div>
    <div class="ekf-row" id="ekf-row">ATT &middot; VEL &middot; POS &middot; HGT</div>
  </div>
</div>

<div class="alt-card">
  <div class="alt-card-title">&#128208; Takeoff Altitude Target</div>
  <div class="alt-row">
    <div>
      <div class="alt-num" id="alt-target-num">1.0</div>
      <div style="font-size:0.68rem;color:var(--text-dim);text-align:right;margin-top:4px">metres</div>
    </div>
    <div class="alt-slider-wrap">
      <input type="range" id="alt-slider" min="3" max="30" value="10" step="1">
      <div class="alt-range-labels"><span>0.3 m</span><span>1.5 m</span><span>3.0 m</span></div>
    </div>
  </div>
  <div class="alt-note">
    Drone will arm, climb in GUIDED, and <strong>automatically switch to LOITER</strong> once it reaches the target altitude.
  </div>
</div>

<div class="controls">
  <button class="btn btn-takeoff" id="btn-takeoff" onclick="send('takeoff')">
    &#x1F680;&nbsp; TAKEOFF &amp; AUTO-HOLD (LOITER)
  </button>
  
  <div class="btn-row">
    <button class="btn btn-loiter" id="btn-loiter" onclick="send('loiter')">
      &#x1F3AF; LOITER / HOLD
    </button>
    <button class="btn btn-land" id="btn-land" onclick="send('land')">
      &#x1F6EC; LAND
    </button>
  </div>

  <div class="btn-row">
    <button class="btn btn-arm"    id="btn-arm"    onclick="send('arm')">&#x26A1; ARM</button>
    <button class="btn btn-disarm" id="btn-disarm" onclick="send('disarm')">&#x2713; DISARM</button>
  </div>
</div>

<div class="toast" id="toast"></div>

<script>
'use strict';
let ws = null, telem = {}, targetAlt = 1.0, toastTimer = null;

const slider = document.getElementById('alt-slider');
const altNum = document.getElementById('alt-target-num');

slider.addEventListener('input', () => {
  targetAlt = (parseInt(slider.value, 10) / 10).toFixed(1);
  altNum.textContent = targetAlt;
});

function connect() {
  ws = new WebSocket('ws://' + location.hostname + ':' + location.port + '/ws');
  ws.onopen  = () => setConn(true);
  ws.onclose = () => { setConn(false); setTimeout(connect, 3000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (e) => {
    const d = JSON.parse(e.data);
    if (d.type === 'telemetry') applyTelem(d);
    else if (d.type === 'ack')  toast(d.msg, d.ok ? 'ok' : 'err');
  };
}

function setConn(ok) {
  document.getElementById('conn-dot').className = 'conn-dot' + (ok ? ' live' : '');
  document.getElementById('conn-text').textContent = ok ? 'FC Connected' : 'Disconnected';
}

function applyTelem(d) {
  telem = d;
  document.getElementById('armed-disp').innerHTML = d.armed
    ? '<span class="badge badge-armed">&#x25CF; Armed</span>'
    : '<span class="badge badge-disarmed">&#x25CF; Disarmed</span>';
  
  const modeEl = document.getElementById('mode-disp');
  modeEl.textContent = d.mode || '\u2014';
  if (d.mode === 'LOITER') modeEl.style.color = 'var(--green)';
  else if (d.mode === 'GUIDED') modeEl.style.color = 'var(--accent)';
  else modeEl.style.color = 'var(--text)';

  document.getElementById('alt-disp').innerHTML =
    (typeof d.alt === 'number' ? d.alt.toFixed(2) : '0.00') + '<span class="tcard-unit">m</span>';
  document.getElementById('ekf-badge-disp').innerHTML = d.ekf_ok
    ? '<span class="badge badge-ekfok">&#x2713; Healthy</span>'
    : '<span class="badge badge-ekfbad">&#x26A0; Not Ready</span>';
  const f = d.ekf_flags || {};
  const c = (ok) => ok ? '#69f0ae' : 'rgba(232,234,246,0.28)';
  document.getElementById('ekf-row').innerHTML =
    `<span style="color:${c(f.att)}">ATT:${f.att?'OK':'--'}</span>` +
    ` &middot; <span style="color:${c(f.vel)}">VEL:${f.vel?'OK':'--'}</span>` +
    ` &middot; <span style="color:${c(f.pos)}">POS:${f.pos?'OK':'--'}</span>` +
    ` &middot; <span style="color:${c(f.hgt)}">HGT:${f.hgt?'OK':'--'}</span>`;

  const armed = d.armed, conn = d.connected;
  document.getElementById('btn-takeoff').disabled = armed || !conn;
  document.getElementById('btn-arm').disabled     = armed || !conn;
  document.getElementById('btn-disarm').disabled  = !armed || !conn;
  document.getElementById('btn-loiter').disabled  = !armed || !conn;
  document.getElementById('btn-land').disabled    = !armed || !conn;
  slider.disabled      = armed;
  slider.style.opacity = armed ? '0.3' : '1';
  altNum.style.color   = armed ? 'var(--text-dim)' : 'var(--accent)';
}

function send(action) {
  if (!ws || ws.readyState !== WebSocket.OPEN) { toast('Not connected to FC', 'err'); return; }
  const p = { action };
  if (action === 'takeoff') p.alt = parseFloat(targetAlt);
  ws.send(JSON.stringify(p));
}

function toast(msg, type) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.className = 'toast visible ' + type;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('visible'), 4000);
}

connect();
</script>
</body>
</html>"""


# ─── Route: Dashboard ─────────────────────────────────────────────────────────
@app.get("/", response_class=HTMLResponse)
async def serve_dashboard():
    return HTMLResponse(content=HTML)


# ─── Startup ──────────────────────────────────────────────────────────────────
@app.on_event("startup")
async def on_startup():
    threading.Thread(target=mavlink_loop, daemon=True).start()
    asyncio.create_task(broadcast_loop())
    try:
        ip = socket.gethostbyname(socket.gethostname())
    except Exception:
        ip = "localhost"
    log.info("=" * 55)
    log.info(f"  Dashboard : http://{ip}:{SERVER_PORT}")
    log.info(f"  Hotspot   : http://{HOTSPOT_IP}:{SERVER_PORT}")
    log.info("=" * 55)


# ─── Entry Point ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    uvicorn.run(
        app,
        host=SERVER_HOST,
        port=SERVER_PORT,
        log_level="warning",
        reload=False,
    )
