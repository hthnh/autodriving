import asyncio
import json
import logging
import os
import threading
import time

import cv2
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, RedirectResponse, StreamingResponse
from serial.tools import list_ports
import uvicorn

from control.intent import ControlIntent, ControlMode
from lane import ColorLaneDetector, HoughLaneDetector, LaneResult
from low_level.client import LowLevelClient
from sensors import CameraManager, LidarManager


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("autodrive-gcs")

# Hardware settings can be overridden without editing this file. When neither
# port is explicit, recognize the common CH340 Nano + CP210x RPLidar pairing so
# Linux enumeration order cannot silently swap their roles.
def _select_serial_ports():
    nano_override = os.getenv("NANO_PORT")
    lidar_override = os.getenv("LIDAR_PORT")
    nano_port = nano_override or "/dev/ttyUSB0"
    lidar_port = lidar_override or "/dev/ttyUSB1"

    try:
        ports = list(list_ports.comports())
    except Exception as exc:
        logger.warning("Could not inspect serial device identities: %s", exc)
        return nano_port, lidar_port

    official_nanos = [
        port for port in ports
        if port.vid in (0x2341, 0x2A03)
        or "arduino" in (port.description or "").lower()
    ]
    ch340_devices = [port for port in ports if port.vid == 0x1A86]
    cp210_devices = [
        port for port in ports
        if port.vid == 0x10C4
        or "cp210" in (port.description or "").lower()
        or "rplidar" in (port.description or "").lower()
    ]

    detected_nano = official_nanos[0].device if len(official_nanos) == 1 else None
    detected_lidar = None
    if len(ch340_devices) == 1 and len(cp210_devices) == 1:
        detected_nano = detected_nano or ch340_devices[0].device
        detected_lidar = cp210_devices[0].device

    if not nano_override and detected_nano:
        nano_port = detected_nano
    if not lidar_override and detected_lidar:
        lidar_port = detected_lidar
    if (nano_port, lidar_port) != ("/dev/ttyUSB0", "/dev/ttyUSB1"):
        logger.info("Serial roles: Nano=%s, RPLidar=%s", nano_port, lidar_port)
    return nano_port, lidar_port


NANO_PORT, LIDAR_PORT = _select_serial_ports()
NANO_BAUD = int(os.getenv("NANO_BAUD", "115200"))
LIDAR_BAUD = int(os.getenv("LIDAR_BAUD", "460800"))
USE_MOCK_LIDAR = os.getenv("USE_MOCK_LIDAR", "auto").strip().lower()
LIDAR_ANGLE_OFFSET_DEG = float(os.getenv("LIDAR_ANGLE_OFFSET_DEG", "0"))
RPLIDAR_SDK_BINARY = os.getenv(
    "RPLIDAR_SDK_BINARY",
    "/home/hthnh2/rplidar_sdk/output/Linux/Release/ultra_simple",
)
FRONT_CAMERA_ID = int(os.getenv("FRONT_CAMERA_ID", "1"))
DOWN_CAMERA_ID = int(os.getenv("DOWN_CAMERA_ID", "0"))
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "480"))
CAMERA_FPS = float(os.getenv("CAMERA_FPS", "15"))
LANE_COLOR_PRESET = os.getenv(
    "LANE_COLOR_PRESET", "black_on_bright"
).strip().lower()
LANE_ROI_Y_START = int(os.getenv("LANE_ROI_Y_START", "280"))
LANE_ROI_Y_END = int(os.getenv("LANE_ROI_Y_END", "480"))
LANE_KP = float(os.getenv("LANE_KP", "0.003"))
LANE_STEER_SIGN = int(os.getenv("LANE_STEER_SIGN", "1"))
LANE_MIN_AREA = int(os.getenv("LANE_MIN_AREA", "300"))
LANE_MORPH_KERNEL = int(os.getenv("LANE_MORPH_KERNEL", "5"))
LANE_BASE_SPEED = float(os.getenv("LANE_BASE_SPEED", "0.12"))
LANE_MIN_CONFIDENCE = float(os.getenv("LANE_MIN_CONFIDENCE", "0.005"))
LANE_LOST_TIMEOUT = float(os.getenv("LANE_LOST_TIMEOUT", "0.3"))
LANE_MAX_STEER_NORM = float(os.getenv("LANE_MAX_STEER_NORM", "0.6"))
LANE_STAGE2_ROI_Y_START = int(os.getenv("LANE_STAGE2_ROI_Y_START", "260"))
LANE_STAGE2_ROI_Y_END = int(os.getenv("LANE_STAGE2_ROI_Y_END", "480"))
LANE_STAGE2_BLUR_KERNEL = int(os.getenv("LANE_STAGE2_BLUR_KERNEL", "5"))
LANE_STAGE2_CANNY_LOW = int(os.getenv("LANE_STAGE2_CANNY_LOW", "50"))
LANE_STAGE2_CANNY_HIGH = int(os.getenv("LANE_STAGE2_CANNY_HIGH", "150"))
LANE_STAGE2_HOUGH_THRESHOLD = int(os.getenv("LANE_STAGE2_HOUGH_THRESHOLD", "30"))
LANE_STAGE2_MIN_LINE_LENGTH = int(os.getenv("LANE_STAGE2_MIN_LINE_LENGTH", "30"))
LANE_STAGE2_MAX_LINE_GAP = int(os.getenv("LANE_STAGE2_MAX_LINE_GAP", "20"))
LANE_STAGE2_SLOPE_MIN_ABS = float(os.getenv("LANE_STAGE2_SLOPE_MIN_ABS", "0.35"))
LANE_STAGE2_SLOPE_MAX_ABS = float(os.getenv("LANE_STAGE2_SLOPE_MAX_ABS", "5.0"))
LANE_STAGE2_LANE_WIDTH_PX = int(os.getenv("LANE_STAGE2_LANE_WIDTH_PX", "220"))
LANE_STAGE2_CENTER_KP = float(os.getenv("LANE_STAGE2_CENTER_KP", "0.003"))
LANE_STAGE2_HEADING_KP = float(os.getenv("LANE_STAGE2_HEADING_KP", "0.015"))
LANE_STAGE2_STEER_SIGN = int(os.getenv("LANE_STAGE2_STEER_SIGN", "1"))
LANE_STAGE2_BASE_SPEED = float(os.getenv("LANE_STAGE2_BASE_SPEED", "0.10"))
LANE_STAGE2_MIN_CONFIDENCE = float(os.getenv("LANE_STAGE2_MIN_CONFIDENCE", "0.15"))
LANE_STAGE2_LOST_TIMEOUT = float(os.getenv("LANE_STAGE2_LOST_TIMEOUT", "0.3"))
LANE_STAGE2_MAX_STEER_NORM = float(os.getenv("LANE_STAGE2_MAX_STEER_NORM", "0.6"))

INPUT_TIMEOUT = 0.3
SPEED_STEP = 0.03
STEER_STEP = 0.05
STEER_RETURN_STEP = 0.02
BROADCAST_PERIOD = 0.1
LIDAR_MAX_POINTS = 360
LIDAR_MAX_DISTANCE_MM = 3000
LIDAR_STALE_SECONDS = 1.0
VEHICLE_LENGTH_MM = 535
VEHICLE_WIDTH_MM = 286

last_input_time = 0.0
speed = 0.0      # -1.0..+1.0
steer = 0.0      # -1.0..+1.0
brake = False
active_controller = None

lane_algorithm_enabled = False
lane_control_enabled = False
lane_active_stage = "stage_1_color_threshold"
AVAILABLE_LANE_STAGES = (
    {"id": ColorLaneDetector.stage_id, "name": ColorLaneDetector.display_name},
    {"id": HoughLaneDetector.stage_id, "name": HoughLaneDetector.display_name},
)
lane_settings = {
    "preset": LANE_COLOR_PRESET,
    "roi_y_start": LANE_ROI_Y_START,
    "roi_y_end": LANE_ROI_Y_END,
    "min_area": LANE_MIN_AREA,
    "morphology_kernel": LANE_MORPH_KERNEL,
    "kp": LANE_KP,
    "steer_sign": LANE_STEER_SIGN,
    "base_speed": LANE_BASE_SPEED,
    "min_confidence": LANE_MIN_CONFIDENCE,
    "lost_lane_timeout": LANE_LOST_TIMEOUT,
    "max_steer_norm": LANE_MAX_STEER_NORM,
}
lane_stage2_settings = {
    "roi_y_start": LANE_STAGE2_ROI_Y_START,
    "roi_y_end": LANE_STAGE2_ROI_Y_END,
    "blur_kernel": LANE_STAGE2_BLUR_KERNEL,
    "canny_low": LANE_STAGE2_CANNY_LOW,
    "canny_high": LANE_STAGE2_CANNY_HIGH,
    "hough_rho": 1.0,
    "hough_theta_deg": 1.0,
    "hough_threshold": LANE_STAGE2_HOUGH_THRESHOLD,
    "min_line_length": LANE_STAGE2_MIN_LINE_LENGTH,
    "max_line_gap": LANE_STAGE2_MAX_LINE_GAP,
    "slope_min_abs": LANE_STAGE2_SLOPE_MIN_ABS,
    "slope_max_abs": LANE_STAGE2_SLOPE_MAX_ABS,
    "lane_width_px": LANE_STAGE2_LANE_WIDTH_PX,
    "center_kp": LANE_STAGE2_CENTER_KP,
    "heading_kp": LANE_STAGE2_HEADING_KP,
    "steer_sign": LANE_STAGE2_STEER_SIGN,
    "base_speed": LANE_STAGE2_BASE_SPEED,
    "min_confidence": LANE_STAGE2_MIN_CONFIDENCE,
    "lost_lane_timeout": LANE_STAGE2_LOST_TIMEOUT,
    "max_steer_norm": LANE_STAGE2_MAX_STEER_NORM,
}
lane_last_update = 0.0
lane_last_stop_time = 0.0

lock = threading.Lock()
clients = set()
broadcast_task = None


def _validate_lane_settings(values):
    """Return a normalized settings copy or raise ValueError."""
    settings = dict(values)
    preset = str(settings["preset"]).strip().lower()
    if preset not in ColorLaneDetector.COLOR_PRESETS:
        raise ValueError("preset must be black_on_bright or white_on_dark")
    settings["preset"] = preset

    for name in ("roi_y_start", "roi_y_end", "min_area", "morphology_kernel"):
        value = settings[name]
        if isinstance(value, bool) or float(value) != int(float(value)):
            raise ValueError(f"{name} must be an integer")
        settings[name] = int(float(value))
    for name in (
        "kp",
        "base_speed",
        "min_confidence",
        "lost_lane_timeout",
        "max_steer_norm",
    ):
        value = settings[name]
        if isinstance(value, bool):
            raise ValueError(f"{name} must be numeric")
        settings[name] = float(value)

    steer_sign = settings["steer_sign"]
    if isinstance(steer_sign, bool) or float(steer_sign) not in (-1.0, 1.0):
        raise ValueError("steer_sign must be -1 or 1")
    settings["steer_sign"] = int(float(steer_sign))

    if settings["roi_y_start"] < 0 or settings["roi_y_end"] <= settings["roi_y_start"]:
        raise ValueError("ROI requires 0 <= roi_y_start < roi_y_end")
    if settings["min_area"] < 0:
        raise ValueError("min_area cannot be negative")
    kernel = settings["morphology_kernel"]
    if kernel <= 0 or kernel % 2 == 0:
        raise ValueError("morphology_kernel must be a positive odd integer")
    if settings["kp"] < 0:
        raise ValueError("kp cannot be negative")
    if not 0.0 <= settings["base_speed"] <= 1.0:
        raise ValueError("base_speed must be between 0 and 1")
    if not 0.0 <= settings["min_confidence"] <= 1.0:
        raise ValueError("min_confidence must be between 0 and 1")
    if settings["lost_lane_timeout"] <= 0:
        raise ValueError("lost_lane_timeout must be positive")
    if not 0.0 <= settings["max_steer_norm"] <= 1.0:
        raise ValueError("max_steer_norm must be between 0 and 1")
    return settings


def _build_lane_detector(settings):
    return ColorLaneDetector(settings=settings)


lane_settings = _validate_lane_settings(lane_settings)
lane_stage2_detector = HoughLaneDetector(settings=lane_stage2_settings)
lane_stage2_settings = dict(lane_stage2_detector.settings)
lane_stage_settings = {
    ColorLaneDetector.stage_id: lane_settings,
    HoughLaneDetector.stage_id: lane_stage2_settings,
}

car = LowLevelClient(
    port=NANO_PORT,
    baud=NANO_BAUD,
    tx_hz=20,
    input_timeout=INPUT_TIMEOUT,
)


lidar_manager = LidarManager(
    port=LIDAR_PORT,
    nano_port=NANO_PORT,
    baud=LIDAR_BAUD,
    mode=USE_MOCK_LIDAR,
    angle_offset_deg=LIDAR_ANGLE_OFFSET_DEG,
    max_points=LIDAR_MAX_POINTS,
    sdk_binary=RPLIDAR_SDK_BINARY,
)
camera_manager = CameraManager(
    front_id=FRONT_CAMERA_ID,
    down_id=DOWN_CAMERA_ID,
    width=CAMERA_WIDTH,
    height=CAMERA_HEIGHT,
    fps=CAMERA_FPS,
)
lane_detectors = {
    ColorLaneDetector.stage_id: _build_lane_detector(lane_settings),
    HoughLaneDetector.stage_id: lane_stage2_detector,
}
lane_detector = lane_detectors[lane_active_stage]
app = FastAPI()


HTML_TEMPLATE = r"""
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Autodrive GCS — Manual Control</title>
<style>
:root { color-scheme: dark; --bg:#090e14; --panel:#121b25; --line:#263748; --text:#e7f2f7; --muted:#8296a5; --cyan:#25d7e8; --green:#45e38a; --amber:#ffc857; --red:#ff5d73; }
* { box-sizing:border-box; }
body { margin:0; min-height:100vh; padding:max(10px,env(safe-area-inset-top)) 10px max(10px,env(safe-area-inset-bottom)); background:radial-gradient(circle at 75% 0,#102535 0,var(--bg) 42%); color:var(--text); font:14px system-ui,-apple-system,Segoe UI,sans-serif; overscroll-behavior:none; }
.topbar { min-height:58px; display:flex; align-items:center; gap:18px; padding:10px 16px; border:1px solid var(--line); border-radius:10px; background:#101922e8; box-shadow:0 10px 30px #0006; }
.brand { margin-right:auto; font-size:19px; font-weight:800; letter-spacing:.12em; color:var(--cyan); white-space:nowrap; }
.nav { display:flex; gap:6px; }
.nav a { padding:7px 10px; border:1px solid var(--line); border-radius:6px; color:var(--muted); text-decoration:none; font-size:11px; font-weight:700; letter-spacing:.08em; }
.nav a.active { color:var(--text); border-color:var(--cyan); background:#12303a; }
.status { color:var(--muted); font-size:12px; letter-spacing:.09em; white-space:nowrap; }
.status b { color:var(--text); margin-left:5px; }
.dot { display:inline-block; width:8px; height:8px; border-radius:50%; margin-right:7px; background:var(--red); box-shadow:0 0 9px currentColor; }
.dot.ok { background:var(--green); }
.dashboard { display:grid; grid-template-columns:minmax(280px,.85fr) minmax(420px,1.4fr); gap:10px; margin-top:10px; }
.column { display:grid; gap:10px; min-width:0; }
.left { grid-template-rows:1fr 1fr; }
.right { grid-template-columns:1.45fr 1fr; grid-template-rows:minmax(390px,1fr) auto; }
.panel { min-width:0; border:1px solid var(--line); border-radius:10px; background:linear-gradient(145deg,#15202b,#101821); box-shadow:0 8px 24px #0005; overflow:hidden; }
.panel-title { display:flex; justify-content:space-between; align-items:center; min-height:39px; padding:8px 12px; border-bottom:1px solid var(--line); color:#b8cad5; font-size:12px; font-weight:700; letter-spacing:.11em; text-transform:uppercase; }
.camera { min-height:260px; display:grid; grid-template-rows:auto 1fr; }
.camera-feed { display:block; width:100%; height:100%; min-height:210px; object-fit:contain; background:#080d12; }
.sensor-error { min-height:18px; padding:3px 10px; color:var(--red); font-size:10px; overflow-wrap:anywhere; }
.lidar-panel { grid-row:1 / 3; display:grid; grid-template-rows:auto minmax(360px,1fr) auto; }
.canvas-wrap { position:relative; min-height:360px; }
#lidarCanvas { display:block; width:100%; height:100%; min-height:360px; }
.lidar-legend { display:grid; grid-template-columns:repeat(3,1fr); gap:1px; border-top:1px solid var(--line); background:var(--line); }
.metric { padding:10px; background:#101821; text-align:center; color:var(--muted); }
.metric b { display:block; margin-top:3px; color:var(--text); font:600 18px ui-monospace,monospace; }
.telemetry, .manual { padding-bottom:10px; }
.telemetry-grid { display:grid; grid-template-columns:repeat(2,1fr); gap:7px; padding:10px; }
.cell { padding:8px; border:1px solid #21313f; border-radius:6px; color:var(--muted); font-size:11px; }
.cell b { display:block; margin-top:3px; color:var(--text); font:600 16px ui-monospace,monospace; }
#raw { margin:0 10px; padding:8px; min-height:38px; overflow-wrap:anywhere; border-radius:5px; background:#080d12; color:#8195a3; font:11px ui-monospace,monospace; }
.control-values { display:grid; grid-template-columns:repeat(5,1fr); gap:7px; padding:10px; text-align:center; }
.controls { display:grid; grid-template-columns:repeat(3,minmax(50px,1fr)); gap:7px; margin:auto; padding:0 10px; max-width:300px; }
button { min-height:48px; border:1px solid #33495c; border-radius:7px; background:#1a2935; color:#dcebf2; font-size:20px; font-weight:800; touch-action:none; user-select:none; -webkit-user-select:none; }
button.active { background:#116b82; border-color:var(--cyan); transform:scale(.97); }
.up { grid-column:2; }.left-btn { grid-column:1;grid-row:2;}.stop { grid-column:2;grid-row:2;min-height:62px;background:#7d172b;border-color:#e64967;font-size:15px;}.right-btn { grid-column:3;grid-row:2;}.down { grid-column:2;grid-row:3; }
.hint { margin:8px 10px 0; color:var(--muted); text-align:center; font-size:11px; }
@media(max-width:900px) { .topbar { flex-wrap:wrap; }.brand { width:100%; }.dashboard { grid-template-columns:1fr; }.left { grid-template-columns:1fr 1fr; grid-template-rows:none; }.right { grid-template-columns:1fr 1fr; }.lidar-panel { grid-column:1 / 3; grid-row:auto; }.camera { min-height:210px; }.camera-feed { min-height:160px; }.control-values { grid-template-columns:repeat(3,1fr); } }
@media(max-width:580px) { .topbar { flex-wrap:wrap; gap:8px 15px; }.brand { width:100%; }.nav { order:2; }.dashboard,.left,.right { display:block; }.panel { margin-bottom:10px; }.lidar-panel { min-height:450px; }.status { font-size:10px; }.control-values { grid-template-columns:repeat(2,1fr); } }
</style>
</head>
<body>
<header class="topbar">
  <div class="brand">AUTODRIVE GCS — MANUAL CONTROL</div>
  <nav class="nav"><a class="active" href="/manual">Manual</a><a href="/autodrive">Autodrive</a></nav>
  <div class="status">MODE <b>MANUAL</b></div>
  <div class="status">STATE <b id="topState">-</b></div>
  <div class="status"><span id="hbDot" class="dot"></span>HB <b id="topHb">WAIT</b></div>
  <div class="status"><span id="wsDot" class="dot"></span>LINK <b id="linkState">OFFLINE</b></div>
</header>

<main class="dashboard">
  <section class="column left">
    <div class="panel camera"><div class="panel-title"><span>Front Camera</span><span id="frontCameraStatus">WAITING</span></div><img class="camera-feed" src="/camera/front.mjpg" alt="Front camera stream"><div id="frontCameraError" class="sensor-error"></div></div>
    <div class="panel camera"><div class="panel-title"><span>Down Camera</span><span id="downCameraStatus">WAITING</span></div><img class="camera-feed" src="/camera/down.mjpg" alt="Down camera stream"><div id="downCameraError" class="sensor-error"></div></div>
  </section>

  <section class="column right">
    <div class="panel lidar-panel">
      <div class="panel-title"><span>Lidar 360°</span><span id="lidarStatus">WAITING</span></div>
      <div class="canvas-wrap"><canvas id="lidarCanvas"></canvas></div>
      <div class="lidar-legend">
        <div class="metric">FRONT ±30°<b id="frontMin">--</b></div>
        <div class="metric">LEFT 30–90°<b id="leftMin">--</b></div>
        <div class="metric">RIGHT −90–−30°<b id="rightMin">--</b></div>
      </div>
      <div id="lidarError" class="sensor-error"></div>
    </div>

    <div class="panel telemetry">
      <div class="panel-title"><span>Telemetry</span><span>10 Hz</span></div>
      <div class="telemetry-grid">
        <div class="cell">RPM<b id="rpm">0</b></div><div class="cell">TARGET<b id="target">0</b></div>
        <div class="cell">RAMP<b id="ramp">0</b></div><div class="cell">PWM<b id="pwm">0</b></div>
        <div class="cell">DIST<b id="dist">0</b></div><div class="cell">HB<b id="hb">0</b></div>
      </div>
      <div id="raw">No telemetry received</div>
    </div>

    <div class="panel manual">
      <div class="panel-title"><span>Manual Control</span><span id="controlSafety">TIMED OUT</span></div>
      <div class="control-values"><div class="cell">SPEED<b id="speed">0.00</b></div><div class="cell">STEER<b id="steer">0.00</b></div><div class="cell">BRAKE<b id="brake">OFF</b></div><div class="cell">INPUT AGE<b id="inputAge">--</b></div><div class="cell">CONTROLLER<b id="activeController">NONE</b></div></div>
      <div class="controls">
        <button class="up" data-key="w" aria-label="Forward">▲</button><button class="left-btn" data-key="a" aria-label="Steer left">◀</button>
        <button class="stop" data-key="space" aria-label="Stop">STOP</button><button class="right-btn" data-key="d" aria-label="Steer right">▶</button><button class="down" data-key="s" aria-label="Reverse">▼</button>
      </div>
      <div class="hint">W/S speed · A/D steer · SPACE stop · release/close triggers safety timeout</div>
    </div>
  </section>
</main>

<script>
const WS_SCHEME = location.protocol === "https:" ? "wss" : "ws";
let ws;
let reconnectTimer;
let keys = {};
let lastLidar = {points:[], timestamp:0, ok:false, source:"waiting"};
const MAX_RADIUS_MM = __LIDAR_MAX_DISTANCE_MM__;
const VEHICLE_LENGTH_MM = __VEHICLE_LENGTH_MM__;
const VEHICLE_WIDTH_MM = __VEHICLE_WIDTH_MM__;
const LIDAR_STALE_SECONDS = __LIDAR_STALE_SECONDS__;
const INPUT_TIMEOUT_SECONDS = __INPUT_TIMEOUT_SECONDS__;
const CONTROL_SEND_INTERVAL_MS = 50;

function connect() {
  ws = new WebSocket(`${WS_SCHEME}://${location.host}/ws`);
  ws.onopen = () => { document.getElementById("wsDot").classList.add("ok"); document.getElementById("linkState").textContent="ONLINE"; };
  ws.onclose = () => { document.getElementById("wsDot").classList.remove("ok"); document.getElementById("linkState").textContent="OFFLINE"; clearTimeout(reconnectTimer); reconnectTimer=setTimeout(connect,1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = event => updateDashboard(JSON.parse(event.data));
}

function sendControls() {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({w:!!keys.w,s:!!keys.s,a:!!keys.a,d:!!keys.d,space:!!keys.space}));
}
document.addEventListener("keydown", event => { const key=event.key===" "?"space":event.key.toLowerCase(); if(["w","s","a","d","space"].includes(key)){event.preventDefault();keys[key]=true;} });
document.addEventListener("keyup", event => { const key=event.key===" "?"space":event.key.toLowerCase(); keys[key]=false; });
function releaseAllControls(){ keys={}; document.querySelectorAll("[data-key]").forEach(b=>b.classList.remove("active")); sendControls(); }
document.querySelectorAll("[data-key]").forEach(button => {
  const key=button.dataset.key;
  button.addEventListener("pointerdown",event=>{event.preventDefault();button.setPointerCapture(event.pointerId);keys[key]=true;button.classList.add("active");sendControls();});
  const release=event=>{event.preventDefault();keys[key]=false;button.classList.remove("active");sendControls();};
  button.addEventListener("pointerup",release);button.addEventListener("pointercancel",release);button.addEventListener("lostpointercapture",release);
});
window.addEventListener("blur",releaseAllControls);document.addEventListener("visibilitychange",()=>{if(document.hidden)releaseAllControls();});
setInterval(sendControls,CONTROL_SEND_INTERVAL_MS);

const setText=(id,value)=>document.getElementById(id).textContent=value ?? "--";
const meters=value=>value==null?"--":`${Number(value).toFixed(2)} m`;
function updateDashboard(data){
  const cmd=data.cmd||{}, tel=data.tel||{};
  setText("speed",Number(cmd.speed||0).toFixed(2));setText("steer",Number(cmd.steer||0).toFixed(2));setText("brake",cmd.brake?"ON":"OFF");
  const hasInputAge=cmd.input_age!==null&&cmd.input_age!==undefined&&Number.isFinite(Number(cmd.input_age));setText("inputAge",hasInputAge?`${Number(cmd.input_age).toFixed(2)} s`:"--");setText("activeController",cmd.active_controller?"ACTIVE":"NONE");
  setText("rpm",tel.rpm);setText("target",tel.target);setText("ramp",tel.ramp);setText("pwm",tel.pwm);setText("dist",tel.dist);setText("hb",tel.hb);setText("raw",tel.raw||"No telemetry received");
  setText("topState",tel.state||"-"); const hbOk=Number(tel.hb)>0; document.getElementById("hbDot").classList.toggle("ok",hbOk);setText("topHb",hbOk?"OK":"LOST");
  const safe=hasInputAge&&Number(cmd.input_age)<INPUT_TIMEOUT_SECONDS;setText("controlSafety",safe?"INPUT LIVE":"TIMED OUT");
  if(data.lidar){lastLidar=data.lidar;setText("frontMin",meters(lastLidar.front_min));setText("leftMin",meters(lastLidar.left_min));setText("rightMin",meters(lastLidar.right_min));setText("lidarError",lastLidar.error||"");}
  for(const name of ["front","down"]){const state=data.camera?.[name];if(!state)continue;const label=state.ok?`${Number(state.fps||0).toFixed(1)} FPS`:`CAM ${state.camera_id} ERROR`;setText(`${name}CameraStatus`,label);setText(`${name}CameraError`,state.error||"");}
}

function drawLidar(){
  const canvas=document.getElementById("lidarCanvas"), rect=canvas.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
  if(canvas.width!==Math.round(rect.width*dpr)||canvas.height!==Math.round(rect.height*dpr)){canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);}
  const ctx=canvas.getContext("2d");ctx.setTransform(dpr,0,0,dpr,0,0);const w=rect.width,h=rect.height,cx=w/2,cy=h/2+5,r=Math.min(w,h)*.42,scale=r/MAX_RADIUS_MM;
  ctx.clearRect(0,0,w,h);ctx.fillStyle="#0b1219";ctx.fillRect(0,0,w,h);
  function sector(start,end,color){ctx.beginPath();ctx.moveTo(cx,cy);for(let a=start;a<=end;a+=2){const rad=a*Math.PI/180;ctx.lineTo(cx+Math.sin(rad)*r,cy-Math.cos(rad)*r);}ctx.closePath();ctx.fillStyle=color;ctx.fill();}
  sector(-30,30,"#25d7e80c");sector(30,90,"#45e38a0a");sector(-90,-30,"#ffc8570a");
  ctx.strokeStyle="#263b4b";ctx.lineWidth=1;ctx.font="10px ui-monospace";ctx.fillStyle="#607887";ctx.textAlign="center";
  for(let m=1;m<=3;m++){ctx.beginPath();ctx.arc(cx,cy,m*1000*scale,0,Math.PI*2);ctx.stroke();ctx.fillText(`${m}m`,cx+4,cy-m*1000*scale+12);}
  ctx.beginPath();ctx.moveTo(cx-r,cy);ctx.lineTo(cx+r,cy);ctx.moveTo(cx,cy-r);ctx.lineTo(cx,cy+r);ctx.stroke();
  ctx.fillStyle="#42dbea";for(const p of lastLidar.points||[]){if(p.distance>MAX_RADIUS_MM)continue;const rad=p.angle*Math.PI/180,x=cx+Math.sin(rad)*p.distance*scale,y=cy-Math.cos(rad)*p.distance*scale;ctx.fillRect(x-1.3,y-1.3,2.6,2.6);}
  const vw=VEHICLE_WIDTH_MM*scale,vl=VEHICLE_LENGTH_MM*scale;ctx.fillStyle="#d7edf2";ctx.strokeStyle="#081015";ctx.lineWidth=2;ctx.fillRect(cx-vw/2,cy-vl/2,vw,vl);ctx.strokeRect(cx-vw/2,cy-vl/2,vw,vl);
  ctx.strokeStyle="#ffca57";ctx.fillStyle="#ffca57";ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(cx,cy-vl/2-5);ctx.lineTo(cx,cy-vl/2-28);ctx.stroke();ctx.beginPath();ctx.moveTo(cx,cy-vl/2-32);ctx.lineTo(cx-5,cy-vl/2-23);ctx.lineTo(cx+5,cy-vl/2-23);ctx.closePath();ctx.fill();
  const stale=!lastLidar.ok||Date.now()/1000-Number(lastLidar.timestamp||0)>LIDAR_STALE_SECONDS;setText("lidarStatus",`${String(lastLidar.source||"unknown").toUpperCase()} · ${stale?"STALE":"LIVE"}`);document.getElementById("lidarStatus").style.color=stale?"var(--red)":"var(--green)";
  requestAnimationFrame(drawLidar);
}
connect();requestAnimationFrame(drawLidar);
</script>
</body>
</html>
"""

HTML = (
    HTML_TEMPLATE
    .replace("__LIDAR_MAX_DISTANCE_MM__", str(LIDAR_MAX_DISTANCE_MM))
    .replace("__VEHICLE_LENGTH_MM__", str(VEHICLE_LENGTH_MM))
    .replace("__VEHICLE_WIDTH_MM__", str(VEHICLE_WIDTH_MM))
    .replace("__LIDAR_STALE_SECONDS__", str(LIDAR_STALE_SECONDS))
    .replace("__INPUT_TIMEOUT_SECONDS__", str(INPUT_TIMEOUT))
)

AUTODRIVE_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Autodrive GCS — Autodrive</title>
<style>
:root{color-scheme:dark;--bg:#090e14;--panel:#121b25;--line:#263748;--text:#e7f2f7;--muted:#8296a5;--cyan:#25d7e8;--green:#45e38a;--amber:#ffc857;--red:#ff5d73}
*{box-sizing:border-box}body{margin:0;min-height:100vh;padding:18px;background:radial-gradient(circle at 75% 0,#102535 0,var(--bg) 42%);color:var(--text);font:14px system-ui,sans-serif}
header{display:flex;align-items:center;gap:18px;padding:15px 18px;border:1px solid var(--line);border-radius:10px;background:#101922}.brand{margin-right:auto;color:var(--cyan);font-size:19px;font-weight:800;letter-spacing:.12em}.nav{display:flex;gap:6px}.nav a{padding:8px 11px;border:1px solid var(--line);border-radius:6px;color:var(--muted);text-decoration:none;font-size:11px;font-weight:700}.nav a.active{color:var(--text);border-color:var(--cyan);background:#12303a}
.link{color:var(--red);font-size:11px;font-weight:700;letter-spacing:.08em}.link.ok{color:var(--green)}
.hero{padding:36px 12px 24px;text-align:center}.hero h1{margin:0;color:var(--cyan);letter-spacing:.14em}.hero p{color:var(--muted)}
.layout{display:grid;grid-template-columns:minmax(360px,1.45fr) minmax(300px,.85fr);gap:12px;max-width:1180px;margin:auto}.card{padding:20px;border:1px solid var(--line);border-radius:10px;background:linear-gradient(145deg,#15202b,#101821);overflow:hidden}.card h2{margin:0 0 15px;font-size:14px;letter-spacing:.08em}.debug{padding:0}.debug h2{margin:0;padding:14px 18px;border-bottom:1px solid var(--line)}.debug img{display:block;width:100%;min-height:300px;object-fit:contain;background:#080d12}
.lane-status{display:flex;align-items:center;gap:10px;margin-bottom:16px;color:var(--red);font-size:23px;font-weight:800}.lane-status.ok{color:var(--green)}.dot{width:11px;height:11px;border-radius:50%;background:currentColor;box-shadow:0 0 10px currentColor}.metrics{display:grid;grid-template-columns:1fr 1fr;gap:9px}.metric{padding:12px;border:1px solid #21313f;border-radius:7px;color:var(--muted);font-size:11px}.metric b{display:block;margin-top:5px;color:var(--text);font:600 16px ui-monospace,monospace}.controls{grid-column:1/-1}.stage-picker{display:flex;align-items:center;gap:12px;margin-bottom:14px}.stage-picker label{color:var(--muted);font-size:11px}.stage-picker select{min-width:260px;padding:8px;border:1px solid #33495c;border-radius:6px;background:#101821;color:var(--text)}.buttons{display:grid;grid-template-columns:repeat(5,1fr);gap:8px}.buttons button,.preset-buttons button{min-height:46px;padding:8px;border:1px solid #33495c;border-radius:7px;background:#1a2935;color:var(--text);font-weight:700;cursor:pointer}.buttons button:hover:not(:disabled),.preset-buttons button:hover{border-color:var(--cyan)}.buttons button:disabled{cursor:not-allowed;opacity:.45}.buttons .danger{background:#70182a;border-color:var(--red)}.tuning-panel{margin-top:14px;padding-top:12px;border-top:1px solid var(--line)}.tuning-panel.hidden{display:none}.tuning-panel h3{margin:0 0 10px;color:#b8cad5;font-size:12px;letter-spacing:.08em}.preset-controls{display:flex;align-items:center;gap:12px;color:var(--muted);font-size:11px}.preset-buttons{display:flex;gap:8px;flex-wrap:wrap}.preset-buttons button{min-height:38px}.settings-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:14px}.setting{display:grid;grid-template-columns:1fr auto;gap:6px;padding:10px;border:1px solid #21313f;border-radius:7px;color:var(--muted);font-size:11px}.setting output{color:var(--cyan);font:12px ui-monospace,monospace}.setting input,.setting select{grid-column:1/-1;width:100%;accent-color:var(--cyan)}.setting select,.setting input[type=number]{padding:6px;border:1px solid #33495c;border-radius:5px;background:#101821;color:var(--text)}.ui-state{margin:12px 0 0;color:var(--amber);font:12px ui-monospace,monospace}.future{grid-column:1/-1}.future-grid{display:grid;grid-template-columns:repeat(5,1fr);gap:8px}.future-item{min-height:76px;padding:12px;border:1px dashed #314657;border-radius:7px;color:var(--muted);font-size:12px}.future-item b{display:block;margin-bottom:5px;color:#aec2cd}.notice{grid-column:1/-1;color:var(--muted);text-align:center;font-size:12px}.notice strong{color:var(--cyan)}
@media(max-width:900px){.buttons,.future-grid,.settings-grid{grid-template-columns:repeat(2,1fr)}}@media(max-width:760px){header{flex-wrap:wrap}.brand{width:100%}.layout{grid-template-columns:1fr}.debug img{min-height:220px}.controls,.future{grid-column:auto}}@media(max-width:460px){.buttons,.future-grid,.metrics,.settings-grid{grid-template-columns:1fr}.preset-controls{align-items:flex-start;flex-direction:column}}
</style>
</head>
<body>
<header><div class="brand">AUTODRIVE GCS</div><nav class="nav"><a href="/manual">Manual</a><a class="active" href="/autodrive">Autodrive</a></nav><span id="link" class="link">OFFLINE</span></header>
<section class="hero"><h1>AUTODRIVE — EDUCATIONAL PIPELINE</h1><p>Compare color and geometric lane detection live; Nano control always requires explicit enablement.</p></section>
<main class="layout">
<article class="card debug"><h2>Rotated Down-Camera Debug</h2><img src="/lane/debug.mjpg" alt="Rotated lane detection debug stream"></article>
<article class="card">
  <h2 id="activeStageTitle">Stage 1 — Color Threshold</h2>
  <div id="laneStatus" class="lane-status"><span class="dot"></span><span id="laneState">LOST</span></div>
  <div class="metrics">
    <div class="metric">ACTIVE STAGE<b id="activeStageMetric">STAGE 1</b></div>
    <div class="metric">ALGORITHM STATUS<b id="algorithmStatus">STOPPED</b></div>
    <div class="metric">CONTROL STATUS<b id="controlStatus">DISABLED</b></div>
    <div class="metric">COLOR PRESET<b id="lanePreset">BLACK ON BRIGHT</b></div>
    <div class="metric">ERROR PX<b id="laneError">--</b></div>
    <div class="metric">HEADING ERROR<b id="headingError">--</b></div>
    <div class="metric">CONFIDENCE<b id="laneConfidence">0.000</b></div>
    <div class="metric">STEER NORM<b id="laneSteer">0.000</b></div>
    <div class="metric">SOURCE CAMERA<b id="laneSource">DOWN</b></div>
    <div class="metric">ROTATION<b id="laneRotation">180°</b></div>
    <div class="metric">COMMANDED SPEED<b id="commandedSpeed">0.000</b></div>
    <div class="metric">COMMANDED STEER<b id="commandedSteer">0.000</b></div>
    <div class="metric">LINES L / R / TOTAL<b id="lineCounts">0 / 0 / 0</b></div>
  </div>
</article>
<article class="card controls">
  <h2>Lane Stage Controls</h2>
  <div class="stage-picker"><label for="activeStage">ACTIVE LANE STAGE</label><select id="activeStage"><option value="stage_1_color_threshold">Stage 1 — Color Threshold</option><option value="stage_2_canny_hough">Stage 2 — Canny + Hough</option></select></div>
  <div class="buttons">
    <button id="startAlgorithm">Start Algorithm</button>
    <button id="stopAlgorithm">Stop Algorithm</button>
    <button id="enableControl">Enable Nano Control</button>
    <button id="disableControl">Disable Nano Control</button>
    <button id="emergencyStop" class="danger">Emergency Stop</button>
  </div>
  <div id="stage1Panel" class="tuning-panel" data-stage-panel="stage_1_color_threshold">
  <h3>Stage 1 Tuning — Color Pixels</h3>
  <div class="preset-controls">
    <span>COLOR PRESET</span>
    <div class="preset-buttons">
      <button id="whitePreset">White Tape / Dark Floor</button>
      <button id="blackPreset">Black Tape / Bright Floor</button>
    </div>
  </div>
  <div class="settings-grid">
    <label class="setting">ROI Y START <output id="roi_y_startValue">280</output><input id="roi_y_start" data-setting="roi_y_start" type="range" min="0" max="470" step="1" value="280"></label>
    <label class="setting">ROI Y END <output id="roi_y_endValue">480</output><input id="roi_y_end" data-setting="roi_y_end" type="range" min="10" max="480" step="1" value="480"></label>
    <label class="setting">MIN AREA <output id="min_areaValue">300</output><input id="min_area" data-setting="min_area" type="range" min="0" max="20000" step="50" value="300"></label>
    <label class="setting">MORPH KERNEL <output id="morphology_kernelValue">5</output><input id="morphology_kernel" data-setting="morphology_kernel" type="range" min="1" max="21" step="2" value="5"></label>
    <label class="setting">KP <output id="kpValue">0.0030</output><input id="kp" data-setting="kp" type="range" min="0" max="0.02" step="0.0001" value="0.003"></label>
    <label class="setting">STEER SIGN <output id="steer_signValue">+1</output><select id="steer_sign" data-setting="steer_sign"><option value="1">+1</option><option value="-1">-1</option></select></label>
    <label class="setting">BASE SPEED <output id="base_speedValue">0.120</output><input id="base_speed" data-setting="base_speed" type="range" min="0" max="0.5" step="0.005" value="0.12"></label>
    <label class="setting">MIN CONFIDENCE <output id="min_confidenceValue">0.005</output><input id="min_confidence" data-setting="min_confidence" type="range" min="0" max="1" step="0.001" value="0.005"></label>
    <label class="setting">LOST TIMEOUT <output id="lost_lane_timeoutValue">0.30 s</output><input id="lost_lane_timeout" data-setting="lost_lane_timeout" type="range" min="0.05" max="2" step="0.05" value="0.3"></label>
    <label class="setting">MAX STEER NORM <output id="max_steer_normValue">0.600</output><input id="max_steer_norm" data-setting="max_steer_norm" type="range" min="0" max="1" step="0.01" value="0.6"></label>
  </div>
  </div>
  <div id="stage2Panel" class="tuning-panel hidden" data-stage-panel="stage_2_canny_hough">
    <h3>Stage 2 Tuning — Canny Edges + Hough Geometry</h3>
    <div class="settings-grid">
      <label class="setting">ROI Y START <output id="s2_roi_y_startValue">260</output><input id="s2_roi_y_start" data-setting="roi_y_start" type="range" min="0" max="470" step="1" value="260"></label>
      <label class="setting">ROI Y END <output id="s2_roi_y_endValue">480</output><input id="s2_roi_y_end" data-setting="roi_y_end" type="range" min="10" max="480" step="1" value="480"></label>
      <label class="setting">BLUR KERNEL <output id="s2_blur_kernelValue">5</output><input id="s2_blur_kernel" data-setting="blur_kernel" type="range" min="1" max="21" step="2" value="5"></label>
      <label class="setting">CANNY LOW <output id="s2_canny_lowValue">50</output><input id="s2_canny_low" data-setting="canny_low" type="range" min="0" max="254" step="1" value="50"></label>
      <label class="setting">CANNY HIGH <output id="s2_canny_highValue">150</output><input id="s2_canny_high" data-setting="canny_high" type="range" min="1" max="255" step="1" value="150"></label>
      <label class="setting">HOUGH THRESHOLD <output id="s2_hough_thresholdValue">30</output><input id="s2_hough_threshold" data-setting="hough_threshold" type="range" min="1" max="150" step="1" value="30"></label>
      <label class="setting">MIN LINE LENGTH <output id="s2_min_line_lengthValue">30</output><input id="s2_min_line_length" data-setting="min_line_length" type="range" min="1" max="300" step="1" value="30"></label>
      <label class="setting">MAX LINE GAP <output id="s2_max_line_gapValue">20</output><input id="s2_max_line_gap" data-setting="max_line_gap" type="range" min="0" max="150" step="1" value="20"></label>
      <label class="setting">SLOPE MIN ABS <output id="s2_slope_min_absValue">0.35</output><input id="s2_slope_min_abs" data-setting="slope_min_abs" type="number" min="0.01" max="10" step="0.01" value="0.35"></label>
      <label class="setting">SLOPE MAX ABS <output id="s2_slope_max_absValue">5.00</output><input id="s2_slope_max_abs" data-setting="slope_max_abs" type="number" min="0.01" max="20" step="0.05" value="5"></label>
      <label class="setting">LANE WIDTH PX <output id="s2_lane_width_pxValue">220</output><input id="s2_lane_width_px" data-setting="lane_width_px" type="range" min="50" max="600" step="5" value="220"></label>
      <label class="setting">CENTER KP <output id="s2_center_kpValue">0.0030</output><input id="s2_center_kp" data-setting="center_kp" type="number" min="0" max="0.05" step="0.0001" value="0.003"></label>
      <label class="setting">HEADING KP <output id="s2_heading_kpValue">0.0150</output><input id="s2_heading_kp" data-setting="heading_kp" type="number" min="0" max="0.1" step="0.0005" value="0.015"></label>
      <label class="setting">STEER SIGN <output id="s2_steer_signValue">+1</output><select id="s2_steer_sign" data-setting="steer_sign"><option value="1">+1</option><option value="-1">-1</option></select></label>
      <label class="setting">BASE SPEED <output id="s2_base_speedValue">0.100</output><input id="s2_base_speed" data-setting="base_speed" type="range" min="0" max="0.5" step="0.005" value="0.1"></label>
      <label class="setting">MIN CONFIDENCE <output id="s2_min_confidenceValue">0.150</output><input id="s2_min_confidence" data-setting="min_confidence" type="range" min="0" max="1" step="0.01" value="0.15"></label>
      <label class="setting">LOST TIMEOUT <output id="s2_lost_lane_timeoutValue">0.30 s</output><input id="s2_lost_lane_timeout" data-setting="lost_lane_timeout" type="number" min="0.05" max="5" step="0.05" value="0.3"></label>
      <label class="setting">MAX STEER NORM <output id="s2_max_steer_normValue">0.600</output><input id="s2_max_steer_norm" data-setting="max_steer_norm" type="range" min="0" max="1" step="0.01" value="0.6"></label>
    </div>
  </div>
  <p id="uiState" class="ui-state">ALGORITHM STOPPED · NANO CONTROL DISABLED</p>
</article>
<article class="card future">
  <h2>Future Modules</h2>
  <div class="future-grid">
    <div class="future-item"><b>Stage 3</b>Perspective Transform</div>
    <div class="future-item"><b>Stage 4</b>Sliding Window + Polynomial</div>
    <div class="future-item"><b>Stage 5</b>Lane Tracking</div>
    <div class="future-item"><b>Lidar</b>Obstacle Avoidance</div>
    <div class="future-item"><b>Fusion</b>Sensor Fusion</div>
    <div class="future-item"><b>Controller</b>Pure Pursuit Controller</div>
    <div class="future-item"><b>Controller</b>Stanley Controller</div>
    <div class="future-item"><b>Controller</b>MPC Controller</div>
    <div class="future-item"><b>Supervisor</b>Safety Supervisor</div>
  </div>
</article>
<p class="notice"><strong>SAFE BY DEFAULT:</strong> Nano control starts disabled and is gated by algorithm state, lane validity, confidence, and manual override.</p>
</main>
<script>
const link=document.getElementById("link"), laneStatus=document.getElementById("laneStatus"), uiState=document.getElementById("uiState");
const setText=(id,value)=>document.getElementById(id).textContent=value;
let ws;
const settingTimers={};
function sendAutodrive(command){
  if(!ws||ws.readyState!==WebSocket.OPEN){uiState.textContent="LINK OFFLINE · COMMAND NOT SENT";return;}
  ws.send(JSON.stringify({autodrive:command}));
}
function formatSetting(id,value){
  const number=Number(value);
  if(["kp","center_kp","heading_kp"].includes(id))return number.toFixed(4);
  if(["slope_min_abs","slope_max_abs"].includes(id))return number.toFixed(2);
  if(["base_speed","min_confidence","max_steer_norm"].includes(id))return number.toFixed(3);
  if(id==="lost_lane_timeout")return `${number.toFixed(2)} s`;
  if(id==="steer_sign")return number>0?"+1":"-1";
  return String(Math.round(number));
}
function syncSettings(settings){
  const panel=document.querySelector("[data-stage-panel]:not(.hidden)");
  if(!panel)return;
  for(const input of panel.querySelectorAll("[data-setting]")){
    const setting=input.dataset.setting;
    if(settings[setting]===undefined)continue;
    if(document.activeElement!==input){input.value=String(settings[setting]);setText(`${input.id}Value`,formatSetting(setting,settings[setting]));}
  }
}
function showStage(stage,available){
  document.getElementById("activeStage").value=stage;
  for(const panel of document.querySelectorAll("[data-stage-panel]"))panel.classList.toggle("hidden",panel.dataset.stagePanel!==stage);
  const item=(available||[]).find(entry=>entry.id===stage),name=item?item.name:stage;
  setText("activeStageTitle",name);setText("activeStageMetric",name.toUpperCase());
}
function connect(){
  const scheme=location.protocol==="https:"?"wss":"ws";
  ws=new WebSocket(`${scheme}://${location.host}/ws`);
  ws.onopen=()=>{link.textContent="ONLINE";link.classList.add("ok")};
  ws.onclose=()=>{link.textContent="OFFLINE";link.classList.remove("ok");setTimeout(connect,1000)};
  ws.onerror=()=>ws.close();
  ws.onmessage=event=>{
    const lane=JSON.parse(event.data).lane||{};
    const ok=lane.ok===true;
    laneStatus.classList.toggle("ok",ok);
    setText("laneState",ok?"OK":"LOST");
    setText("laneError",lane.error_px==null?"--":Number(lane.error_px).toFixed(1));
    setText("headingError",lane.heading_error_deg==null?"--":`${Number(lane.heading_error_deg).toFixed(1)}°`);
    setText("laneConfidence",Number(lane.confidence||0).toFixed(3));
    setText("laneSteer",Number(lane.steer_norm||0).toFixed(3));
    setText("lanePreset",lane.preset?String(lane.preset).replace(/_/g," ").toUpperCase():"N/A");
    setText("laneSource",String(lane.source_camera||"none").toUpperCase());
    setText("laneRotation",lane.rotated_180?"180°":"NONE");
    setText("algorithmStatus",lane.algorithm_enabled?"RUNNING":"STOPPED");
    setText("controlStatus",lane.control_enabled?"ENABLED":"DISABLED");
    setText("commandedSpeed",Number(lane.commanded_speed||0).toFixed(3));
    setText("commandedSteer",Number(lane.commanded_steer||0).toFixed(3));
    setText("lineCounts",`${Number(lane.left_line_count||0)} / ${Number(lane.right_line_count||0)} / ${Number(lane.total_line_count||0)}`);
    document.getElementById("enableControl").disabled=!lane.algorithm_enabled;
    showStage(lane.active_stage,lane.available_stages);
    syncSettings(lane.settings||{});
    uiState.textContent=`${lane.algorithm_enabled?"ALGORITHM RUNNING":"ALGORITHM STOPPED"} · NANO CONTROL ${lane.control_enabled?"ENABLED":"DISABLED"}`;
  };
}
document.getElementById("startAlgorithm").onclick=()=>sendAutodrive({algorithm_enabled:true});
document.getElementById("stopAlgorithm").onclick=()=>sendAutodrive({algorithm_enabled:false,control_enabled:false});
document.getElementById("enableControl").onclick=()=>sendAutodrive({control_enabled:true});
document.getElementById("disableControl").onclick=()=>sendAutodrive({control_enabled:false});
document.getElementById("emergencyStop").onclick=()=>sendAutodrive({emergency_stop:true});
document.getElementById("activeStage").onchange=event=>sendAutodrive({active_stage:event.target.value});
document.getElementById("whitePreset").onclick=()=>sendAutodrive({settings:{preset:"white_on_dark"}});
document.getElementById("blackPreset").onclick=()=>sendAutodrive({settings:{preset:"black_on_bright"}});
for(const input of document.querySelectorAll("[data-setting]")){
  const setting=input.dataset.setting;
  const update=()=>{
    setText(`${input.id}Value`,formatSetting(setting,input.value));
    const panel=input.closest("[data-stage-panel]");
    const roiStart=Number(panel.querySelector('[data-setting="roi_y_start"]').value),roiEnd=Number(panel.querySelector('[data-setting="roi_y_end"]').value);
    if(roiStart>=roiEnd){uiState.textContent="ROI START MUST BE LESS THAN ROI END";return;}
    const low=panel.querySelector('[data-setting="canny_low"]'),high=panel.querySelector('[data-setting="canny_high"]');
    if(low&&high&&Number(low.value)>=Number(high.value)){uiState.textContent="CANNY LOW MUST BE LESS THAN CANNY HIGH";return;}
    const slopeMin=panel.querySelector('[data-setting="slope_min_abs"]'),slopeMax=panel.querySelector('[data-setting="slope_max_abs"]');
    if(slopeMin&&slopeMax&&Number(slopeMin.value)>Number(slopeMax.value)){uiState.textContent="SLOPE MIN MUST NOT EXCEED SLOPE MAX";return;}
    clearTimeout(settingTimers[input.id]);
    settingTimers[input.id]=setTimeout(()=>sendAutodrive({settings:{[setting]:Number(input.value)}}),100);
  };
  input.addEventListener(input.tagName==="SELECT"?"change":"input",update);
}
connect();
</script>
</body>
</html>
"""


@app.get("/")
async def index():
    return RedirectResponse(url="/manual", status_code=307)


@app.get("/manual")
async def manual_page():
    return HTMLResponse(HTML)


@app.get("/autodrive")
async def autodrive_page():
    return HTMLResponse(AUTODRIVE_HTML)


async def camera_stream(name):
    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    delay = 1.0 / max(1.0, CAMERA_FPS)
    while True:
        frame = camera_manager.get_frame(name)
        yield boundary + frame + b"\r\n"
        await asyncio.sleep(delay)


async def lane_debug_stream():
    """Stream the rotated down-camera overlay without publishing an intent."""
    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    delay = 1.0 / max(1.0, CAMERA_FPS)
    while True:
        frame_rgb = camera_manager.get_array("down")

        if frame_rgb is None:
            # CameraWorker already provides a labeled JPEG placeholder.
            jpeg = camera_manager.get_frame("down")
        else:
            try:
                rotated_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)
                with lock:
                    detector = lane_detector
                    active_stage = lane_active_stage
                    algorithm_enabled = lane_algorithm_enabled
                if algorithm_enabled:
                    result = detector.detect(rotated_rgb)
                else:
                    result = LaneResult(
                        ok=False,
                        stage_id=active_stage,
                        lane_center_x=None,
                        image_center_x=rotated_rgb.shape[1] // 2,
                        error_px=None,
                        confidence=0.0,
                        contour_area=0.0,
                        steer_norm=0.0,
                    )
                debug_rgb = detector.draw_debug(rotated_rgb, result)
                debug_bgr = cv2.cvtColor(debug_rgb, cv2.COLOR_RGB2BGR)
                encoded, jpeg_array = cv2.imencode(
                    ".jpg", debug_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80]
                )
                if not encoded:
                    raise RuntimeError("lane debug JPEG encoding failed")
                jpeg = jpeg_array.tobytes()
            except Exception as exc:
                logger.warning("Lane debug frame failed: %s", exc)
                jpeg = camera_manager.get_frame("down")

        yield boundary + jpeg + b"\r\n"
        await asyncio.sleep(delay)


@app.get("/camera/{name}.mjpg")
async def camera_endpoint(name: str):
    if name not in ("front", "down"):
        return HTMLResponse("Camera not found", status_code=404)
    return StreamingResponse(
        camera_stream(name),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


@app.get("/lane/debug.mjpg")
async def lane_debug_endpoint():
    return StreamingResponse(
        lane_debug_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


def handle_autodrive_input(data):
    """Apply live active-stage settings and safety-gated enable commands."""
    global lane_algorithm_enabled, lane_control_enabled
    global lane_active_stage, lane_detector, lane_settings, lane_last_stop_time
    global lane_detectors, lane_stage_settings
    if not isinstance(data, dict):
        raise TypeError("autodrive payload must be an object")
    for name in ("algorithm_enabled", "control_enabled", "emergency_stop"):
        if name in data and not isinstance(data[name], bool):
            raise TypeError(f"{name} must be boolean")

    stop_intent = None
    now = time.time()
    with lock:
        if bool(data.get("emergency_stop")):
            lane_algorithm_enabled = False
            lane_control_enabled = False
            lane_last_stop_time = now
            stop_intent = ControlIntent.stop("autodrive_emergency")
        else:
            previous_control = lane_control_enabled
            target_stage = data.get("active_stage", lane_active_stage)
            if target_stage not in lane_stage_settings:
                raise ValueError("unknown lane stage")
            stage_switched = target_stage != lane_active_stage
            target_settings = lane_stage_settings[target_stage]
            target_detector = lane_detectors[target_stage]
            updates = data.get("settings")
            if updates is not None:
                if not isinstance(updates, dict):
                    raise TypeError("autodrive settings must be an object")
                candidate = dict(target_settings)
                candidate.update(updates)
                if target_stage == ColorLaneDetector.stage_id:
                    candidate = _validate_lane_settings(candidate)
                    target_detector = _build_lane_detector(candidate)
                else:
                    target_detector = HoughLaneDetector(settings=candidate)
                    candidate = dict(target_detector.settings)
                target_settings = candidate

            # Commit the validated detector/settings and stage as one state
            # transition so a bad combined command cannot partially switch.
            lane_stage_settings[target_stage] = target_settings
            lane_detectors[target_stage] = target_detector
            if stage_switched:
                lane_active_stage = target_stage
                lane_control_enabled = False
                lane_last_stop_time = now
                stop_intent = ControlIntent.stop("stage_switch")
            lane_settings = lane_stage_settings[lane_active_stage]
            lane_detector = lane_detectors[lane_active_stage]

            if "algorithm_enabled" in data:
                lane_algorithm_enabled = data["algorithm_enabled"]

            if "control_enabled" in data:
                requested = data["control_enabled"]
                manual_is_active = (
                    active_controller is not None
                    and now - last_input_time <= INPUT_TIMEOUT
                )
                lane_control_enabled = bool(
                    requested
                    and lane_algorithm_enabled
                    and not manual_is_active
                    and not stage_switched
                )

            if not lane_algorithm_enabled:
                lane_control_enabled = False

            if lane_control_enabled and not previous_control:
                lane_last_stop_time = 0.0

            if previous_control and not lane_control_enabled and not stage_switched:
                lane_last_stop_time = now
                stop_intent = ControlIntent.stop("lane_disabled")

    if stop_intent is not None:
        car.set_intent(stop_intent)
    return True


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            data = json.loads(await ws.receive_text())
            if isinstance(data, dict) and "autodrive" in data:
                handle_autodrive_input(data["autodrive"])
            else:
                handle_input(data, ws)
    except (ValueError, TypeError):
        logger.warning("Closing websocket after invalid control payload")
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        logger.warning("Websocket closed with error: %s", exc)
    finally:
        clients.discard(ws)


def handle_input(data, controller=None):
    global speed, steer, brake, last_input_time, active_controller
    global lane_control_enabled
    if not isinstance(data, dict):
        raise TypeError("control payload must be an object")

    now = time.time()
    has_action = any(bool(data.get(key)) for key in ("w", "s", "a", "d", "space"))
    has_action = has_action or data.get("tap") in ("w", "s", "a", "d", "space")

    with lock:
        if active_controller is not None and now - last_input_time > INPUT_TIMEOUT:
            speed = 0.0
            steer = 0.0
            brake = True
            active_controller = None

        # An idle tab must never refresh another tab's safety heartbeat. A tab
        # becomes the controller only after an explicit key/button action.
        if active_controller is None:
            if controller is not None and not has_action:
                return False
            active_controller = controller
        elif active_controller is not controller:
            return False

        # An accepted explicit manual action always overrides lane control.
        if has_action:
            lane_control_enabled = False

        last_input_time = now
        if data.get("space") or data.get("tap") == "space":
            speed = 0.0
            brake = True
        else:
            brake = False
            if data.get("w") or data.get("tap") == "w":
                speed += SPEED_STEP
            if data.get("s") or data.get("tap") == "s":
                speed -= SPEED_STEP

        if data.get("a") or data.get("tap") == "a":
            steer -= STEER_STEP
        elif data.get("d") or data.get("tap") == "d":
            steer += STEER_STEP
        elif steer > 0:
            steer -= STEER_RETURN_STEP
        elif steer < 0:
            steer += STEER_RETURN_STEP

        speed = max(-1.0, min(1.0, speed))
        steer = max(-1.0, min(1.0, steer))
        if abs(steer) < 0.03:
            steer = 0.0

        intent = ControlIntent(
            speed=speed,
            steer=steer,
            brake=brake,
            mode=ControlMode.MANUAL,
            source="web",
        )
    car.set_intent(intent)
    return True


def expire_stale_controller(now=None):
    """Release a disconnected/quiet controller and publish an explicit stop."""
    global speed, steer, brake, active_controller
    now = time.time() if now is None else now
    expired = False
    with lock:
        if active_controller is not None and now - last_input_time > INPUT_TIMEOUT:
            speed = 0.0
            steer = 0.0
            brake = True
            active_controller = None
            expired = True
    if expired:
        car.set_intent(ControlIntent.stop("web_input_timeout"))
    return expired


def process_lane_stage1(now=None):
    """Run one lane cycle and publish a command only when safety gates pass."""
    global lane_last_update, lane_last_stop_time
    now = time.time() if now is None else now
    with lock:
        active_stage = lane_active_stage
        algorithm_enabled = lane_algorithm_enabled
        control_enabled = lane_control_enabled
        settings = dict(lane_settings)
        detector = lane_detector

    result = LaneResult(
        ok=False,
        stage_id=active_stage,
        lane_center_x=None,
        image_center_x=CAMERA_WIDTH // 2,
        error_px=None,
        confidence=0.0,
        contour_area=0.0,
        steer_norm=0.0,
    )
    frame_rgb = camera_manager.get_array("down")
    if algorithm_enabled and frame_rgb is not None:
        try:
            rotated_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)
            result = detector.detect(rotated_rgb)
        except Exception as exc:
            logger.warning("Lane detection failed: %s", exc)

    lane_is_valid = bool(
        algorithm_enabled
        and result.ok
        and result.confidence >= settings["min_confidence"]
    )
    commanded_speed = 0.0
    commanded_steer = 0.0

    if control_enabled and lane_is_valid:
        commanded_speed = settings["base_speed"]
        commanded_steer = max(
            -settings["max_steer_norm"],
            min(settings["max_steer_norm"], result.steer_norm),
        )
        car.set_intent(ControlIntent(
            speed=commanded_speed,
            steer=commanded_steer,
            brake=False,
            mode=ControlMode.LANE_KEEP,
            source=active_stage,
        ))
        lane_last_stop_time = 0.0
    elif control_enabled:
        # Stop immediately on loss, then reassert the stop at the configured
        # timeout while loss persists. No stale lane command is continued.
        if (
            lane_last_stop_time == 0.0
            or now - lane_last_stop_time >= settings["lost_lane_timeout"]
        ):
            car.set_intent(ControlIntent.stop("lane_lost"))
            lane_last_stop_time = now

    lane_last_update = now
    return {
        "stage": active_stage,
        "active_stage": active_stage,
        "available_stages": list(AVAILABLE_LANE_STAGES),
        "enabled": algorithm_enabled,
        "algorithm_enabled": algorithm_enabled,
        "control_enabled": control_enabled,
        "settings": settings,
        "ok": result.ok,
        "error_px": result.error_px,
        "heading_error_deg": result.heading_error_deg,
        "confidence": result.confidence,
        "left_line_count": result.left_line_count,
        "right_line_count": result.right_line_count,
        "total_line_count": result.total_line_count,
        "steer_norm": result.steer_norm,
        "commanded_speed": commanded_speed,
        "commanded_steer": commanded_steer,
        "source_camera": "down",
        "rotated_180": True,
        "preset": settings.get("preset"),
        "last_update": lane_last_update,
        "stage_debug": result.debug,
    }


async def broadcast_thread():
    while True:
        expire_stale_controller()
        with lock:
            command = {
                "speed": speed,
                "steer": steer,
                "brake": brake,
                "input_age": (
                    time.time() - last_input_time
                    if last_input_time
                    else None
                ),
                "active_controller": active_controller is not None,
            }

        lane_data = process_lane_stage1()
        payload = json.dumps({
            "cmd": command,
            "tel": car.get_telemetry(),
            "lidar": lidar_manager.get_state(),
            "camera": camera_manager.get_state(),
            "lane": lane_data,
        })
        recipients = list(clients)
        results = await asyncio.gather(
            *(_send_payload(ws, payload) for ws in recipients),
            return_exceptions=False,
        )
        dead = [ws for ws, sent in zip(recipients, results) if not sent]
        for ws in dead:
            clients.discard(ws)
        await asyncio.sleep(BROADCAST_PERIOD)


async def _send_payload(ws, payload):
    try:
        await asyncio.wait_for(
            ws.send_text(payload),
            timeout=BROADCAST_PERIOD,
        )
        return True
    except Exception:
        return False


@app.on_event("startup")
async def startup():
    global broadcast_task
    car.start()
    lidar_manager.start()
    camera_manager.start()
    if broadcast_task is None or broadcast_task.done():
        broadcast_task = asyncio.create_task(
            broadcast_thread(),
            name="websocket-broadcast",
        )


@app.on_event("shutdown")
async def shutdown():
    global broadcast_task, active_controller
    if broadcast_task is not None:
        broadcast_task.cancel()
        try:
            await broadcast_task
        except asyncio.CancelledError:
            pass
        broadcast_task = None

    for ws in list(clients):
        try:
            await ws.close(code=1001)
        except Exception:
            pass
    clients.clear()

    car.close()
    camera_manager.stop()
    lidar_manager.stop()
    with lock:
        active_controller = None


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
