# core/camera_manager.py

import threading
import time
from picamera2 import Picamera2


class CameraDevice:
    def __init__(self, cam_id, config):
        self.cam_id = cam_id
        self.config = config

        self.picam2 = None
        self.thread = None
        self.running = False

        self.latest_frame = None
        self.lock = threading.Lock()

    # ---------- START ----------
    def start(self):
        if self.running:
            return

        self.picam2 = Picamera2(camera_num=self.cam_id)

        self.picam2.preview_configuration.main.size = self.config["size"]
        self.picam2.preview_configuration.main.format = "RGB888"
        self.picam2.preview_configuration.controls.FrameRate = self.config["fps"]

        self.picam2.configure("preview")
        self.picam2.start()

        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

        print(f"[CAM {self.cam_id}] Started")

    # ---------- STOP ----------
    def stop(self):
        if not self.running:
            return

        self.running = False
        self.thread.join()

        self.picam2.stop()
        self.picam2.close()
        self.picam2 = None

        print(f"[CAM {self.cam_id}] Stopped")

    # ---------- LOOP ----------
    def _capture_loop(self):
        while self.running:
            frame = self.picam2.capture_array()

            with self.lock:
                self.latest_frame = frame

    # ---------- GET FRAME ----------
    def get_frame(self):
        with self.lock:
            return self.latest_frame


# ======================================================


class CameraManager:
    def __init__(self):
        self.devices = {}

    def add_camera(self, cam_id, config):
        self.devices[cam_id] = CameraDevice(cam_id, config)

    def start(self, cam_id):
        if cam_id in self.devices:
            self.devices[cam_id].start()

    def stop(self, cam_id):
        if cam_id in self.devices:
            self.devices[cam_id].stop()

    def get_frame(self, cam_id):
        if cam_id in self.devices:
            return self.devices[cam_id].get_frame()
        return None

    def is_running(self, cam_id):
        if cam_id in self.devices:
            return self.devices[cam_id].running
        return False
