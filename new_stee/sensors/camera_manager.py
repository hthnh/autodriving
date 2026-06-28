import base64
import io
import logging
import threading
import time


logger = logging.getLogger("autodrive-gcs.camera")

# Valid tiny JPEG used only if neither OpenCV nor Pillow can generate a labeled
# placeholder. Camera status still carries the human-readable failure reason.
FALLBACK_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAP//////////////////////////////////////////////////////////////////////////////////////2wBDAf//////////////////////////////////////////////////////////////////////////////////////wAARCAABAAEDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAX/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIQAxAAAAF//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQABBQJ//8QAFBEBAAAAAAAAAAAAAAAAAAAAAP/aAAgBAwEBPwF//8QAFBEBAAAAAAAAAAAAAAAAAAAAAP/aAAgBAgEBPwF//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQAGPwJ//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQABPyF//9oADAMBAAIAAwAAABD/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oACAEDAQE/EH//xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oACAECAQE/EH//xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oACAEBAAE/EH//2Q=="
)


class CameraWorker:
    def __init__(self, name, camera_id, width, height, target_fps):
        self.name = name
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.target_fps = target_fps

        self.running = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.camera = None
        self.frame = self._placeholder("camera starting")
        self.state = {
            "ok": False,
            "fps": 0.0,
            "error": "camera starting",
            "timestamp": 0.0,
            "camera_id": camera_id,
        }

    def start(self):
        if self.running.is_set():
            return
        self.running.set()
        self.thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"camera-{self.name}",
        )
        self.thread.start()

    def stop(self):
        self.running.clear()
        camera = self.camera
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=3.0)
        self.thread = None

    def get_frame(self):
        with self.lock:
            return self.frame

    def get_state(self):
        with self.lock:
            return dict(self.state)

    def _capture_loop(self):
        try:
            from picamera2 import Picamera2

            self.camera = Picamera2(camera_num=self.camera_id)
            config = self.camera.create_video_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"},
                controls={"FrameRate": self.target_fps},
                buffer_count=4,
            )
            self.camera.configure(config)
            self.camera.start()
            logger.info(
                "%s camera %d started at %dx%d @ %.1f FPS",
                self.name,
                self.camera_id,
                self.width,
                self.height,
                self.target_fps,
            )

            frames = 0
            sample_started = time.monotonic()
            measured_fps = 0.0
            while self.running.is_set():
                array = self.camera.capture_array("main")
                jpeg = self._encode(array)
                frames += 1
                elapsed = time.monotonic() - sample_started
                if elapsed >= 1.0:
                    measured_fps = frames / elapsed
                    frames = 0
                    sample_started = time.monotonic()
                now = time.time()
                with self.lock:
                    self.frame = jpeg
                    self.state = {
                        "ok": True,
                        "fps": round(measured_fps, 1),
                        "error": "",
                        "timestamp": now,
                        "camera_id": self.camera_id,
                    }
        except Exception as exc:
            error = f"camera {self.camera_id} unavailable: {exc}"
            logger.warning("%s camera: %s", self.name, error)
            with self.lock:
                self.frame = self._placeholder(error)
                self.state = {
                    "ok": False,
                    "fps": 0.0,
                    "error": error,
                    "timestamp": time.time(),
                    "camera_id": self.camera_id,
                }
        finally:
            camera, self.camera = self.camera, None
            if camera is not None:
                try:
                    camera.stop()
                except Exception:
                    pass
                try:
                    camera.close()
                except Exception:
                    pass

    @staticmethod
    def _encode(array):
        try:
            import cv2

            ok, encoded = cv2.imencode(
                ".jpg",
                array,
                [cv2.IMWRITE_JPEG_QUALITY, 80],
            )
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            return encoded.tobytes()
        except ImportError:
            from PIL import Image

            output = io.BytesIO()
            Image.fromarray(array).save(output, format="JPEG", quality=80)
            return output.getvalue()

    def _placeholder(self, error):
        label = self.name.upper() + " CAMERA"
        detail = error[:72]
        try:
            import cv2
            import numpy as np

            image = np.full((self.height, self.width, 3), (24, 32, 41), np.uint8)
            cv2.putText(image, label, (28, self.height // 2 - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 235, 242), 2)
            cv2.putText(image, detail, (28, self.height // 2 + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (95, 130, 150), 1)
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                return encoded.tobytes()
        except ImportError:
            pass

        try:
            from PIL import Image, ImageDraw

            image = Image.new("RGB", (self.width, self.height), (24, 32, 41))
            draw = ImageDraw.Draw(image)
            draw.text((28, self.height // 2 - 20), label, fill=(220, 235, 242))
            draw.text((28, self.height // 2 + 8), detail, fill=(95, 130, 150))
            output = io.BytesIO()
            image.save(output, format="JPEG", quality=75)
            return output.getvalue()
        except ImportError:
            return FALLBACK_JPEG


class CameraManager:
    """Own two independent CSI camera workers and their latest JPEG frames."""

    def __init__(self, front_id=0, down_id=1, width=640, height=480, fps=15.0):
        self.fps = fps
        self.cameras = {
            "front": CameraWorker("front", front_id, width, height, fps),
            "down": CameraWorker("down", down_id, width, height, fps),
        }

    def start(self):
        for camera in self.cameras.values():
            camera.start()

    def stop(self):
        for camera in self.cameras.values():
            camera.stop()

    def get_frame(self, name):
        return self.cameras[name].get_frame()

    def get_state(self):
        return {
            name: camera.get_state()
            for name, camera in self.cameras.items()
        }
