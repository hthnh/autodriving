import asyncio
import json
import logging
import os
import threading
import time

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, RedirectResponse, StreamingResponse
from serial.tools import list_ports
import uvicorn

from control.intent import ControlIntent, ControlMode
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
FRONT_CAMERA_ID = int(os.getenv("FRONT_CAMERA_ID", "0"))
DOWN_CAMERA_ID = int(os.getenv("DOWN_CAMERA_ID", "1"))
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "480"))
CAMERA_FPS = float(os.getenv("CAMERA_FPS", "15"))

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

lock = threading.Lock()
clients = set()
broadcast_task = None

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
:root{color-scheme:dark;--bg:#090e14;--panel:#121b25;--line:#263748;--text:#e7f2f7;--muted:#8296a5;--cyan:#25d7e8}
*{box-sizing:border-box}body{margin:0;min-height:100vh;padding:18px;background:radial-gradient(circle at 75% 0,#102535 0,var(--bg) 42%);color:var(--text);font:14px system-ui,sans-serif}
header{display:flex;align-items:center;gap:18px;padding:15px 18px;border:1px solid var(--line);border-radius:10px;background:#101922}.brand{margin-right:auto;color:var(--cyan);font-size:19px;font-weight:800;letter-spacing:.12em}.nav{display:flex;gap:6px}.nav a{padding:8px 11px;border:1px solid var(--line);border-radius:6px;color:var(--muted);text-decoration:none;font-size:11px;font-weight:700}.nav a.active{color:var(--text);border-color:var(--cyan);background:#12303a}
.hero{padding:54px 12px 34px;text-align:center}.hero h1{margin:0;color:var(--cyan);letter-spacing:.14em}.hero p{color:var(--muted)}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;max-width:1100px;margin:auto}.card{min-height:150px;padding:20px;border:1px solid var(--line);border-radius:10px;background:linear-gradient(145deg,#15202b,#101821)}.card h2{margin-top:0;font-size:14px;letter-spacing:.08em}.card p{color:var(--muted)}@media(max-width:760px){header{flex-wrap:wrap}.brand{width:100%}.grid{grid-template-columns:1fr}}
</style>
</head>
<body>
<header><div class="brand">AUTODRIVE GCS</div><nav class="nav"><a href="/manual">Manual</a><a class="active" href="/autodrive">Autodrive</a></nav></header>
<section class="hero"><h1>AUTODRIVE MODE — COMING NEXT</h1><p>Page structure only. No autonomous control is enabled.</p></section>
<main class="grid">
<article class="card"><h2>Lane Detection</h2><p>Camera lane geometry and confidence.</p></article>
<article class="card"><h2>Follow Target</h2><p>Target selection, range, and tracking state.</p></article>
<article class="card"><h2>Obstacle Avoidance</h2><p>Lidar sectors and collision constraints.</p></article>
<article class="card"><h2>Planner State</h2><p>Route, behavior, and maneuver state.</p></article>
<article class="card"><h2>Control Intent</h2><p>Requested speed, steering, and braking.</p></article>
<article class="card"><h2>Safety Supervisor</h2><p>Heartbeat, timeout, and override state.</p></article>
</main>
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


@app.get("/camera/{name}.mjpg")
async def camera_endpoint(name: str):
    if name not in ("front", "down"):
        return HTMLResponse("Camera not found", status_code=404)
    return StreamingResponse(
        camera_stream(name),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            msg = await ws.receive_text()
            handle_input(json.loads(msg), ws)
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
        payload = json.dumps({
            "cmd": command,
            "tel": car.get_telemetry(),
            "lidar": lidar_manager.get_state(),
            "camera": camera_manager.get_state(),
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
