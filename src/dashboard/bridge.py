#!/usr/bin/env python3
"""
Dashboard Bridge — ROSLess Version
====================================
Reads drone state directly from the MAVLink TCP bridge (start_drone.py)
and the LiDAR obstacle binary (/tmp/drone_obstacles.bin).

No ROS required.

Run:
    python3 dashboard/bridge.py
Dashboard:
    http://<PI_IP>:8765
"""
import asyncio, json, math, os, subprocess, threading, time
from pathlib import Path

import numpy as np
from pymavlink import mavutil
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

# ─────────────────────────────────────────────────────────────────────────────
BRIDGE_HOST   = os.environ.get("FC_BRIDGE_HOST", "127.0.0.1")
BRIDGE_PORT   = int(os.environ.get("FC_BRIDGE_PORT", "5760"))
DASHBOARD_HOST = os.environ.get("DASHBOARD_HOST", "0.0.0.0")
DASHBOARD_PORT = int(os.environ.get("DASHBOARD_PORT", "8765"))
LIDAR_FILE    = "/tmp/drone_obstacles.bin"
DASHBOARD_DIR = Path(__file__).parent
SECTORS       = 72
SECTOR_DEG    = 5.0
MAX_LOG       = 25

COPTER_MODES = {
    "STABILIZE":0,"ALT_HOLD":2,"AUTO":3,"GUIDED":4,
    "LOITER":5,"RTL":6,"LAND":9,"GUIDED_NOGPS":20,
}
MODE_ID_NAME = {v:k for k,v in COPTER_MODES.items()}

# ─────────────────────────────────────────────────────────────────────────────
shared = {
    "drone":  {"x":0.0,"y":0.0,"z":0.0,"roll":0.0,"pitch":0.0,"yaw":0.0,"vx":0.0,"vy":0.0,"vz":0.0},
    "fc":     {"armed":False,"mode":"UNKNOWN","connected":False,"battery_pct":None,"battery_v":None},
    "sensors":{"floor_m":None,"ceil_m":None,"flow_quality":None,"lidar_pts":0},
    "log":    [],
}
_lock = threading.Lock()
mav_conn = None

def _log(sev, msg):
    with _lock:
        shared["log"].append({"t":round(time.time(),2),"sev":sev,"msg":msg})
        if len(shared["log"]) > MAX_LOG:
            shared["log"].pop(0)

def mavlink_thread():
    global mav_conn
    _retry_count = 0
    while True:
        try:
            if _retry_count == 0:
                _log("info", "Connecting to MAVLink bridge (start_drone.py)…")
            mav = mavutil.mavlink_connection(
                f"tcp:{BRIDGE_HOST}:{BRIDGE_PORT}",
                source_system=253, source_component=0, autoreconnect=True)
            hb = mav.wait_heartbeat(timeout=15)
            if hb is None:
                _log("warn","No heartbeat — is start_drone.py running?")
                _retry_count += 1
                time.sleep(10); continue
            _retry_count = 0  # reset on success
            mav_conn = mav
            with _lock: shared["fc"]["connected"] = True
            _log("info",f"FC connected sys={mav.target_system}")
            for sid in [mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                        mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS]:
                mav.mav.request_data_stream_send(mav.target_system,mav.target_component,sid,10,1)
            last_hb = time.monotonic()
            while True:
                msg = mav.recv_match(blocking=True,timeout=0.5)
                if msg is None:
                    if time.monotonic()-last_hb > 5.0:
                        _log("warn","FC heartbeat lost"); 
                        with _lock: shared["fc"]["connected"]=False
                        break
                    continue
                t = msg.get_type()
                if t=="HEARTBEAT" and msg.get_srcSystem()==mav.target_system:
                    last_hb = time.monotonic()
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mode  = MODE_ID_NAME.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
                    with _lock: shared["fc"]["armed"]=armed; shared["fc"]["mode"]=mode; shared["fc"]["connected"]=True
                elif t=="ATTITUDE":
                    with _lock:
                        shared["drone"]["roll"]  = round(math.degrees(msg.roll),1)
                        shared["drone"]["pitch"] = round(math.degrees(msg.pitch),1)
                        shared["drone"]["yaw"]   = round(math.degrees(msg.yaw),1)
                elif t=="LOCAL_POSITION_NED":
                    with _lock:
                        shared["drone"]["x"]  = round(msg.x,3)
                        shared["drone"]["y"]  = round(msg.y,3)
                        shared["drone"]["z"]  = round(-msg.z,3)
                        shared["drone"]["vx"] = round(msg.vx,3)
                        shared["drone"]["vy"] = round(msg.vy,3)
                        shared["drone"]["vz"] = round(-msg.vz,3)
                elif t=="VFR_HUD":
                    with _lock:
                        if shared["drone"]["z"]==0.0: shared["drone"]["z"]=round(msg.alt,2)
                elif t=="DISTANCE_SENSOR":
                    d = round(msg.current_distance/100.0,3)
                    with _lock:
                        if msg.orientation==25: shared["sensors"]["floor_m"]=d
                        elif msg.orientation==24: shared["sensors"]["ceil_m"]=d
                elif t=="OPTICAL_FLOW_RAD":
                    with _lock: shared["sensors"]["flow_quality"]=int(msg.quality)
                elif t=="SYS_STATUS":
                    v = msg.voltage_battery/1000.0
                    if v>0:
                        pct = max(0.0,min(100.0,(v-13.2)/(16.8-13.2)*100.0))
                        with _lock: shared["fc"]["battery_v"]=round(v,2); shared["fc"]["battery_pct"]=round(pct,1)
                elif t=="STATUSTEXT":
                    txt=msg.text.rstrip("\x00").strip()
                    if txt:
                        sev={6:"info",4:"warn",3:"warn",2:"error",0:"error"}.get(msg.severity,"info")
                        _log(sev,txt)
        except Exception as exc:
            if _retry_count == 0:
                # Log the first failure only — don't spam on repeated retries
                print(f"[bridge] MAVLink unavailable: {exc} — will retry every 10s", flush=True)
            mav_conn=None
            with _lock: shared["fc"]["connected"]=False
            _retry_count += 1
        time.sleep(10)  # slow retry — start_drone.py may not be running yet

def read_lidar():
    try:
        data = np.fromfile(LIDAR_FILE, dtype=np.uint16)
        if len(data) != SECTORS: return []
        result=[]
        for i,d in enumerate(data):
            if 20 < int(d) < 800: result.append([round(i*SECTOR_DEG,1), round(int(d)/100.0,2)])
        with _lock: shared["sensors"]["lidar_pts"]=len(result)
        return result
    except: return []

SENSORS_FILE = "/tmp/drone_sensors.json"
_last_sensors: dict = {}

def read_sensors_file() -> dict:
    """Read sensor data written by start_drone.py sensor_export_thread (10 Hz)."""
    global _last_sensors
    try:
        with open(SENSORS_FILE) as f:
            data = json.load(f)
        age = time.time() - data.get("t", 0)
        if age < 3.0:           # only use if fresh (wall-clock time, safe cross-process)
            _last_sensors = data
    except Exception:
        pass
    return _last_sensors

# ─────────────────────────────────────────────────────────────────────────────
app = FastAPI(title="Drone Dashboard")
app.mount("/static", StaticFiles(directory=str(DASHBOARD_DIR/"static")), name="static")

@app.get("/")
async def root():
    return HTMLResponse((DASHBOARD_DIR/"index.html").read_text())


def _build_frame() -> dict:
    """Build a complete JSON-serialisable state frame to push to the browser."""
    with _lock:
        frame = json.loads(json.dumps(shared))
    frame["lidar"] = read_lidar()
    sf = read_sensors_file()
    if sf:
        if sf.get("floor_m")      is not None: frame["sensors"]["floor_m"]     = sf["floor_m"]
        if sf.get("ceil_m")       is not None: frame["sensors"]["ceil_m"]      = sf["ceil_m"]
        if sf.get("flow_quality") is not None: frame["sensors"]["flow_quality"] = sf["flow_quality"]
        if sf.get("lidar_scans")  is not None: frame["sensors"]["lidar_pts"]   = sf["lidar_scans"]
    return frame


@app.websocket("/ws")
async def ws_ep(ws: WebSocket):
    await ws.accept()
    _log("info", "WebSocket client connected")

    # Background receiver — handles incoming commands
    async def _recv_loop():
        while True:
            try:
                txt = await ws.receive_text()
            except WebSocketDisconnect:
                return
            except Exception:
                return
            try:
                await handle_cmd(json.loads(txt))
            except Exception:
                pass  # bad JSON or cmd error — keep receiving

    recv_task = asyncio.create_task(_recv_loop())
    try:
        while True:
            # Build and send frame — if this fails the connection is gone
            try:
                frame = _build_frame()
            except Exception as exc:
                _log("error", f"_build_frame error: {exc}")
                import traceback; traceback.print_exc()
                break
            try:
                await ws.send_json(frame)
            except WebSocketDisconnect:
                break
            except Exception as exc:
                _log("error", f"WS send error: {exc}")
                break
            await asyncio.sleep(0.1)
    finally:
        recv_task.cancel()
        _log("info", "WebSocket client disconnected")

# ── GUIDED takeoff helpers ──────────────────────

def _set_fake_ekf_origin(mav_ref):
    """
    Injects a fake GPS global origin (0, 0, 0) into ArduPilot.
    This creates the local Cartesian map grid required to unlock standard GUIDED mode
    on a drone with no physical GPS module.
    """
    _log("info", "Injecting fake EKF origin to establish indoor local grid…")
    mav_ref.mav.set_gps_global_origin_send(
        mav_ref.target_system,
        0,                          # latitude  (int32, degrees × 1E7)
        0,                          # longitude (int32, degrees × 1E7)
        0,                          # altitude  (int32, mm AMSL)
        int(time.time() * 1000),    # time_usec (ms timestamp)
    )


async def _guided_takeoff_task(mav_ref, target_alt: float):
    """
    Full GUIDED-mode takeoff sequence:
      1. Inject fake EKF origin so ArduPilot has a local coordinate grid.
      2. Switch to GUIDED mode (uses full EKF3 with optical flow + lidar).
      3. Arm motors.
      4. Send MAV_CMD_NAV_TAKEOFF to target altitude.
      5. Monitor rangefinder altitude; log when target is reached.
    """
    _log("info", f"GUIDED takeoff: climbing to {target_alt:.1f} m")
    try:
        # Step 1 — Establish indoor map origin
        _set_fake_ekf_origin(mav_ref)
        await asyncio.sleep(2.0)   # Give EKF3 time to ingest the origin

        # Step 2 — Switch to GUIDED mode (custom_mode = 4)
        _log("info", "Setting mode → GUIDED")
        mav_ref.mav.command_long_send(
            mav_ref.target_system, mav_ref.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            float(COPTER_MODES["GUIDED"]), 0, 0, 0, 0, 0,
        )
        await asyncio.sleep(1.0)

        # Step 3 — Arm
        _log("info", "Arming motors…")
        mav_ref.mav.command_long_send(
            mav_ref.target_system, mav_ref.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0,   # param1=1 → ARM
        )
        await asyncio.sleep(1.0)

        # Step 4 — MAV_CMD_NAV_TAKEOFF
        _log("info", f"Sending NAV_TAKEOFF to {target_alt:.1f} m…")
        mav_ref.mav.command_long_send(
            mav_ref.target_system, mav_ref.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, target_alt,  # param7 = target altitude (m)
        )

        # Step 5 — Monitor altitude until target reached (30 s timeout)
        deadline = asyncio.get_event_loop().time() + 30.0
        while asyncio.get_event_loop().time() < deadline:
            with _lock:
                z = shared["drone"]["z"]
            if z >= target_alt * 0.9:
                _log("info", f"GUIDED takeoff: reached {z:.2f} m — hovering")
                break
            await asyncio.sleep(0.2)
        else:
            _log("warn", f"GUIDED takeoff: timeout — current alt {z:.2f} m")

    except asyncio.CancelledError:
        _log("info", "GUIDED takeoff cancelled")
    except Exception as exc:
        _log("error", f"GUIDED takeoff error: {exc}")


_TAKEOFF_TASK: asyncio.Task = None


async def handle_cmd(data):
    global _TAKEOFF_TASK
    mav=mav_conn
    if mav is None: return
    cmd=data.get("type","")
    def _mode(name):
        mid=COPTER_MODES.get(name,20)
        mav.mav.command_long_send(mav.target_system,mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,float(mid),0,0,0,0,0)
    def _arm(v):
        mav.mav.command_long_send(mav.target_system,mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,float(v),0,0,0,0,0,0)
    if cmd=="arm":
        # Arm in GUIDED mode so EKF3 is active from the very first moment
        _mode("GUIDED"); await asyncio.sleep(0.4); _arm(1)
    elif cmd=="disarm": _arm(0)
    elif cmd=="set_mode": _mode(data.get("mode","GUIDED"))
    elif cmd=="takeoff":
        alt = float(data.get("altitude", 1.0))
        if _TAKEOFF_TASK and not _TAKEOFF_TASK.done():
            _TAKEOFF_TASK.cancel(); await asyncio.sleep(0.1)
        _TAKEOFF_TASK = asyncio.create_task(_guided_takeoff_task(mav, alt))
    elif cmd=="land":
        if _TAKEOFF_TASK and not _TAKEOFF_TASK.done():
            _TAKEOFF_TASK.cancel()
        _mode("LAND")
    elif cmd=="rtl":  _mode("RTL")
    elif cmd=="loiter": _mode("LOITER")


if __name__=="__main__":
    threading.Thread(target=mavlink_thread,daemon=True).start()
    time.sleep(1.5)
    try: ip=subprocess.check_output("hostname -I|awk '{print $1}'",shell=True).decode().strip()
    except: ip="localhost"
    print(f"\n{'='*55}\n  Dashboard : http://{ip}:{DASHBOARD_PORT}\n  Requires  : src/start_drone.py running\n{'='*55}\n")
    uvicorn.run(app, host=DASHBOARD_HOST, port=DASHBOARD_PORT, log_level="warning")
