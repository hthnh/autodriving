import time
import cv2
import json
import asyncio

from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi import Request

from control.hub import ControlHub

app = FastAPI()
hub = ControlHub()

app.mount("/static", StaticFiles(directory="web_gui/static"), name="static")
templates = Jinja2Templates(directory="web_gui/templates")


# ==============================
# HTML
# ==============================

@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})


# ==============================
# STATUS (WebSocket realtime)
# ==============================

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            status = hub.get_vehicle_status()
            follow = hub.get_follow_status()
            lane = hub.get_lane_status()

            data = {
                "vehicle": status,
                "follow": follow,
                "lane": lane
            }

            await ws.send_text(json.dumps(data))
            await asyncio.sleep(0.1)

    except Exception:
        print("WebSocket disconnected")



# ==============================
# CONTROL API
# ==============================

@app.post("/set_mode/{mode}")
def set_mode(mode: str):
    hub.set_mode(mode)
    return {"ok": True}


@app.post("/set_level/{level}")
def set_level(level: int):
    hub.set_level(level)
    return {"ok": True}


@app.post("/set_input/{speed}/{steer}")
def set_input(speed: int, steer: int):
    hub.set_input(speed, steer)
    return {"ok": True}


@app.post("/emergency")
def emergency():
    hub.emergency_stop()
    return {"ok": True}


# ==============================
# CAMERA STREAM
# ==============================

def generate_camera_stream(cam_id):
    while True:
        frame = hub.get_camera_frame(cam_id)

        if frame is None:
            time.sleep(0.05)
            continue

        _, jpeg = cv2.imencode(".jpg", frame)
        frame_bytes = jpeg.tobytes()

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" +
               frame_bytes + b"\r\n")

        time.sleep(0.03)


@app.get("/camera/{cam_id}")
def camera_stream(cam_id: int):
    return StreamingResponse(
        generate_camera_stream(cam_id),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.post("/camera/{cam_id}/start")
def start_camera(cam_id: int):
    hub.start_camera(cam_id)
    return {"ok": True}

@app.post("/camera/{cam_id}/stop")
def stop_camera(cam_id: int):
    hub.stop_camera(cam_id)
    return {"ok": True}

