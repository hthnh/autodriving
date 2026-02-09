import time
import numpy as np
import cv2
from picamera2 import Picamera2


class LaneAlgoGeometry:
    def __init__(
        self,
        state,
        img_width=640,
        img_height=480,
        car_center_offset_px=0
    ):
        self.state = state

        self.img_width = img_width
        self.img_height = img_height

        self.image_center_x = img_width / 2
        self.car_center_x = self.image_center_x + car_center_offset_px

        self.running = False
        self.picam2 = None

    # ---------- LIFECYCLE ----------
    def start(self):
        if self.running:
            return

        self.picam2 = Picamera2(camera_num=0)
        self.picam2.preview_configuration.main.size = (
            self.img_width, self.img_height
        )
        self.picam2.preview_configuration.main.format = "RGB888"
        self.picam2.preview_configuration.controls.FrameRate = 30
        self.picam2.configure("preview")
        self.picam2.start()

        self.running = True
        print("[LANE] Picamera2 started")

    def stop(self):
        if not self.running:
            return

        self.picam2.stop()
        self.picam2.close()
        self.picam2 = None

        self.running = False
        self.state.error = 0.0
        self.state.confidence = 0.0
        print("[LANE] Picamera2 stopped")

    # ---------- MAIN UPDATE ----------
    def update(self):
        if not self.running or self.picam2 is None:
            return

        frame = self.picam2.capture_array()   # RGB888
        h, w, _ = frame.shape

        # --------- GEOMETRY SIMPLE PIPELINE ---------
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # Lấy histogram đáy ảnh để ước lượng lane
        bottom = edges[int(h * 0.7):, :]
        hist = np.sum(bottom, axis=0)

        mid = w // 2
        left_x = np.argmax(hist[:mid])
        right_x = np.argmax(hist[mid:]) + mid

        lane_center = (left_x + right_x) / 2
        error_px = lane_center - self.car_center_x
        lane_width = abs(right_x - left_x)

        if lane_width < 50:
            self.state.confidence = 0.0
            return

        error = error_px / (lane_width / 2)
        error = max(-1.0, min(1.0, error))

        # --------- UPDATE STATE ---------
        self.state.error = float(error)
        self.state.confidence = 1.0
        self.state.timestamp = time.time()

        self.state.debug_frames = {
            "raw": frame,
            "edges": edges
        }

        self.state.variables = {
            "left_x": left_x,
            "right_x": right_x,
            "lane_center": lane_center,
            "car_center": self.car_center_x,
            "error": error
        }
