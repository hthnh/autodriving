import asyncio
import json
import time
import threading

from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
import uvicorn

from control.intent import ControlIntent, ControlMode
from low_level.client import LowLevelClient


last_input_time = 0.0
INPUT_TIMEOUT = 0.3

speed = 0.0      # -1.0..+1.0
steer = 0.0      # -1.0..+1.0
brake = False

lock = threading.Lock()
clients = set()

car = LowLevelClient(
    port="/dev/ttyUSB0",
    baud=115200,
    tx_hz=20,
    input_timeout=INPUT_TIMEOUT,
)

app = FastAPI()



HTML = """
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Autodrive Manual</title>
<style>
* {
    box-sizing: border-box;
}
body {
    font-family: Arial;
    background: #111;
    color: #eee;
    text-align: center;
    margin: 0;
    min-height: 100vh;
    padding: max(12px, env(safe-area-inset-top)) 12px
             max(12px, env(safe-area-inset-bottom));
    overscroll-behavior: none;
}
.card {
    background: #222;
    margin: 12px auto;
    padding: 18px;
    border-radius: 12px;
    width: min(100%, 480px);
}
.value {
    font-size: clamp(22px, 7vw, 30px);
    margin: 8px;
}
button {
    border: 0;
    border-radius: 14px;
    background: #444;
    color: white;
    font-size: 30px;
    font-weight: bold;
    min-height: 72px;
    touch-action: none;
    user-select: none;
    -webkit-user-select: none;
    -webkit-tap-highlight-color: transparent;
}
button.active {
    background: #1976d2;
    transform: scale(0.96);
}
.controls {
    display: grid;
    grid-template-columns: repeat(3, minmax(72px, 1fr));
    gap: 10px;
    margin: 16px auto 0;
    max-width: 390px;
}
.up { grid-column: 2; grid-row: 1; }
.left { grid-column: 1; grid-row: 2; }
.stop { grid-column: 2; grid-row: 2; }
.right { grid-column: 3; grid-row: 2; }
.down { grid-column: 2; grid-row: 3; }
.hint {
    color: #bbb;
    margin: 4px 0;
}
.stop {
    background: #b00020;
    color: white;
}
.stop.active {
    background: #ff1744;
}
.telemetry-grid {
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 8px;
    text-align: left;
}
#raw {
    overflow-wrap: anywhere;
    font-family: monospace;
    font-size: 13px;
}
@media (orientation: landscape) and (max-height: 600px) {
    body {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 12px;
    }
    .card { margin: 0 auto; }
    h1, h2 { margin: 4px 0; }
    button { min-height: 58px; }
}
</style>
</head>
<body>
<div class="card">
    <h1>Autodrive Manual</h1>
    <p class="hint">Chạm và giữ để điều khiển</p>
    <p class="hint">W/S speed · A/D steer · SPACE stop</p>

    <div class="value">Speed: <span id="speed">0.00</span></div>
    <div class="value">Steer: <span id="steer">0.00</span></div>

    <div class="controls">
        <button class="up" data-key="w" aria-label="Forward">▲</button>
        <button class="left" data-key="a" aria-label="Steer left">◀</button>
        <button class="stop" data-key="space" aria-label="Stop">STOP</button>
        <button class="right" data-key="d" aria-label="Steer right">▶</button>
        <button class="down" data-key="s" aria-label="Reverse">▼</button>
    </div>
</div>

<div class="card">
    <h2>Telemetry</h2>
    <div class="telemetry-grid">
        <div>STATE: <span id="state">-</span></div>
        <div>RPM: <span id="rpm">0</span></div>
        <div>TARGET: <span id="target">0</span></div>
        <div>RAMP: <span id="ramp">0</span></div>
        <div>PWM: <span id="pwm">0</span></div>
        <div>DIST: <span id="dist">0</span></div>
        <div>HB: <span id="hb">0</span></div>
    </div>
    <hr>
    <div id="raw"></div>
</div>

<script>
let ws = new WebSocket(`ws://${location.host}/ws`);

let keys = {};

function sendControls() {
    if (ws.readyState !== WebSocket.OPEN) return;

    ws.send(JSON.stringify({
        w: keys["w"] || false,
        s: keys["s"] || false,
        a: keys["a"] || false,
        d: keys["d"] || false,
        space: keys["space"] || false
    }));
}

document.addEventListener("keydown", e => {
    let key = e.key === " " ? "space" : e.key.toLowerCase();
    if (["w", "s", "a", "d", "space"].includes(key)) {
        e.preventDefault();
        keys[key] = true;
    }
});

document.addEventListener("keyup", e => {
    let key = e.key === " " ? "space" : e.key.toLowerCase();
    keys[key] = false;
});

function releaseAllControls() {
    keys = {};
    document.querySelectorAll("[data-key]").forEach(button => {
        button.classList.remove("active");
    });
    sendControls();
}

document.querySelectorAll("[data-key]").forEach(button => {
    let key = button.dataset.key;

    button.addEventListener("pointerdown", event => {
        event.preventDefault();
        button.setPointerCapture(event.pointerId);
        keys[key] = true;
        button.classList.add("active");
        sendControls();
    });

    function release(event) {
        event.preventDefault();
        keys[key] = false;
        button.classList.remove("active");
        sendControls();
    }

    button.addEventListener("pointerup", release);
    button.addEventListener("pointercancel", release);
    button.addEventListener("lostpointercapture", release);
});

window.addEventListener("blur", releaseAllControls);
document.addEventListener("visibilitychange", () => {
    if (document.hidden) releaseAllControls();
});

setInterval(sendControls, 50);

ws.onmessage = ev => {
    let d = JSON.parse(ev.data);

    document.getElementById("speed").innerText = d.cmd.speed.toFixed(2);
    document.getElementById("steer").innerText = d.cmd.steer.toFixed(2);

    document.getElementById("state").innerText = d.tel.state;
    document.getElementById("rpm").innerText = d.tel.rpm;
    document.getElementById("target").innerText = d.tel.target;
    document.getElementById("ramp").innerText = d.tel.ramp;
    document.getElementById("pwm").innerText = d.tel.pwm;
    document.getElementById("dist").innerText = d.tel.dist;
    document.getElementById("hb").innerText = d.tel.hb;
    document.getElementById("raw").innerText = d.tel.raw;
};
</script>
</body>
</html>
"""


@app.get("/")
def index():
    return HTMLResponse(HTML)


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)

    try:
        while True:
            msg = await ws.receive_text()
            handle_input(json.loads(msg))
    except Exception:
        pass
    finally:
        clients.discard(ws)


def handle_input(data):
    global speed, steer, brake
    global last_input_time
    last_input_time = time.time()

    with lock:
        if data.get("space") or data.get("tap") == "space":
            speed = 0.0
            brake = True
        else:
            brake = False

            if data.get("w") or data.get("tap") == "w":
                speed += 0.03

            if data.get("s") or data.get("tap") == "s":
                speed -= 0.03

        if data.get("a") or data.get("tap") == "a":
            steer -= 0.05
        elif data.get("d") or data.get("tap") == "d":
            steer += 0.05
        else:
            if steer > 0:
                steer -= 0.02
            elif steer < 0:
                steer += 0.02

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


async def broadcast_thread():
    while True:
        with lock:
            command = {
                "speed": speed,
                "steer": steer,
                "brake": brake,
                "input_age": time.time() - last_input_time,
            }

        payload = json.dumps({
            "cmd": command,
            "tel": car.get_telemetry(),
        })

        dead = []

        for ws in clients:
            try:
                await ws.send_text(payload)
            except Exception:
                dead.append(ws)

        for ws in dead:
            clients.discard(ws)

        await asyncio.sleep(0.1)


@app.on_event("startup")
async def startup():
    car.start()
    asyncio.create_task(broadcast_thread())


@app.on_event("shutdown")
async def shutdown():
    car.close()


if __name__ == "__main__":
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000
    )
