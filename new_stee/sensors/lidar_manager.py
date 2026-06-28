import logging
import math
import os
from pathlib import Path
import pty
import re
import select
import signal
import subprocess
import threading
import time


logger = logging.getLogger("autodrive-gcs.lidar")

MOCK_VALUES = {"1", "true", "yes", "on", "mock"}
REAL_VALUES = {"0", "false", "no", "off", "real"}


class LidarManager:
    """Own one RPLidar connection and publish compact, thread-safe scans."""

    def __init__(
        self,
        port,
        nano_port,
        baud=460800,
        mode="auto",
        angle_offset_deg=0.0,
        max_points=360,
        update_period=0.15,
        sdk_binary="",
    ):
        self.port = port
        self.nano_port = nano_port
        self.baud = baud
        self.mode = str(mode).strip().lower()
        self.angle_offset_deg = angle_offset_deg
        self.max_points = max_points
        self.update_period = update_period
        self.sdk_binary = sdk_binary

        self.running = threading.Event()
        self.thread = None
        self.lock = threading.Lock()
        self.lidar = None
        self.process = None
        self.state = self._empty_state("starting", "")

    @staticmethod
    def _empty_state(source, error):
        return {
            "ok": False,
            "source": source,
            "points": [],
            "front_min": None,
            "left_min": None,
            "right_min": None,
            "timestamp": 0.0,
            "error": error,
        }

    def start(self):
        if self.running.is_set():
            return
        self.running.set()
        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="lidar-reader",
        )
        self.thread.start()

    def stop(self):
        self.running.clear()
        self._interrupt_lidar()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=10.0)
        if self.thread and self.thread.is_alive():
            logger.error("Lidar reader did not terminate cleanly")
        else:
            self.thread = None

    def get_state(self):
        with self.lock:
            return {
                **self.state,
                "points": [dict(point) for point in self.state["points"]],
            }

    def _set_error(self, error):
        logger.warning("%s", error)
        with self.lock:
            self.state = self._empty_state("error", error)
            self.state["timestamp"] = time.time()

    def _set_scan(self, source, points, error=""):
        normalized = self._normalize_points(points)
        state = {
            "ok": bool(normalized),
            "source": source,
            "points": normalized,
            "front_min": self._sector_min(normalized, -30, 30),
            "left_min": self._sector_min(normalized, 30, 90),
            "right_min": self._sector_min(normalized, -90, -30),
            "timestamp": time.time(),
            "error": error,
        }
        with self.lock:
            self.state = state

    @staticmethod
    def _signed_angle(angle):
        return (angle + 180.0) % 360.0 - 180.0

    @classmethod
    def _sector_min(cls, points, low, high):
        distances = [
            point["distance"]
            for point in points
            if low <= cls._signed_angle(point["angle"]) <= high
        ]
        return round(min(distances) / 1000.0, 3) if distances else None

    def _normalize_points(self, points):
        bins = {}
        for angle, distance in points:
            try:
                angle = (float(angle) + self.angle_offset_deg) % 360.0
                distance = float(distance)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(distance) or distance <= 0:
                continue
            degree = int(round(angle)) % 360
            previous = bins.get(degree)
            if previous is None or distance < previous:
                bins[degree] = distance
        return [
            {"angle": degree, "distance": round(bins[degree], 1)}
            for degree in sorted(bins)[:self.max_points]
        ]

    def _run(self):
        if self.mode in MOCK_VALUES:
            logger.info("Lidar source: mock (forced)")
            self._run_mock()
            return

        error = self._real_configuration_error()
        if not error:
            error = self._run_real_drivers()

        if not self.running.is_set():
            return
        if self.mode in REAL_VALUES:
            self._set_error(error or "lidar unavailable")
            return

        logger.warning("%s; falling back to mock lidar", error)
        self._run_mock(error=error)

    def _run_real_drivers(self):
        python_error = ""
        try:
            self._run_rplidar()
            if not self.running.is_set():
                return ""
            python_error = "Python lidar scan ended"
        except ModuleNotFoundError:
            python_error = "rplidar package not installed"
        except Exception as exc:
            python_error = f"Python lidar driver failed: {exc}"
        finally:
            self._close_lidar()

        if not self.running.is_set():
            return ""
        if not self.sdk_binary or not Path(self.sdk_binary).is_file():
            return python_error
        try:
            logger.info("Using Slamtec SDK lidar driver: %s", self.sdk_binary)
            self._run_sdk()
            return "" if not self.running.is_set() else "Slamtec SDK scan ended"
        except Exception as exc:
            return f"{python_error}; Slamtec SDK driver failed: {exc}"

    def _real_configuration_error(self):
        if self._ports_conflict():
            return f"lidar port conflicts with Nano port: {self.port}"
        if not Path(self.port).exists():
            return "lidar port not found"
        return ""

    def _ports_conflict(self):
        try:
            return Path(self.port).resolve() == Path(self.nano_port).resolve()
        except OSError:
            return self.port == self.nano_port

    def _run_rplidar(self):
        from rplidar import RPLidar

        logger.info("Opening RPLidar on %s at %d baud", self.port, self.baud)
        self.lidar = RPLidar(self.port, baudrate=self.baud, timeout=3)
        self.lidar.clean_input()
        info = self.lidar.get_info()
        self.lidar.clean_input()
        if info.get("model") == 65 and self.sdk_binary:
            raise RuntimeError("model 65 requires Slamtec SDK ultra scan")
        for scan in self.lidar.iter_scans(max_buf_meas=1000):
            if not self.running.is_set():
                break
            points = [
                (measurement[1], measurement[2])
                for measurement in scan
                if len(measurement) >= 3
            ]
            if points:
                self._set_scan("rplidar", points)

    def _run_sdk(self):
        command = [
            self.sdk_binary,
            "--channel",
            "--serial",
            self.port,
            str(self.baud),
        ]
        master_fd, slave_fd = pty.openpty()
        try:
            self.process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=slave_fd,
                stderr=slave_fd,
                close_fds=True,
                start_new_session=True,
            )
        finally:
            os.close(slave_fd)

        pattern = re.compile(
            r"theta:\s*([0-9.]+)\s+Dist:\s*([0-9.]+)\s+Q:\s*(\d+)"
        )
        buffer = ""
        revolution = []
        last_angle = None
        shutdown_started = None
        kill_sent = False
        try:
            while True:
                return_code = self.process.poll()
                if return_code is not None:
                    if shutdown_started is not None:
                        return
                    raise RuntimeError(f"SDK process exited with code {return_code}")

                if not self.running.is_set() and shutdown_started is None:
                    shutdown_started = time.monotonic()
                    self._signal_process(signal.SIGINT)
                if (
                    shutdown_started is not None
                    and not kill_sent
                    and time.monotonic() - shutdown_started > 8.0
                ):
                    kill_sent = True
                    self._signal_process(signal.SIGKILL)

                readable, _, _ = select.select([master_fd], [], [], 0.1)
                if not readable:
                    continue
                try:
                    chunk = os.read(master_fd, 65536).decode(errors="ignore")
                except OSError:
                    continue
                if not chunk or shutdown_started is not None:
                    continue
                buffer += chunk
                lines = buffer.split("\n")
                buffer = lines.pop()
                for line in lines:
                    match = pattern.search(line)
                    if not match:
                        continue
                    angle = float(match.group(1))
                    distance = float(match.group(2))
                    quality = int(match.group(3))
                    if last_angle is not None and angle < last_angle - 180:
                        if revolution:
                            self._set_scan("rplidar", revolution)
                        revolution = []
                    if distance > 0 and quality > 0:
                        revolution.append((angle, distance))
                    last_angle = angle
        finally:
            os.close(master_fd)
            process, self.process = self.process, None
            if process is not None and process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def _interrupt_lidar(self):
        lidar = self.lidar
        if lidar is not None:
            for method_name in ("stop", "stop_motor", "disconnect"):
                try:
                    method = getattr(lidar, method_name, None)
                    if method:
                        method()
                except Exception:
                    pass

    def _signal_process(self, sig):
        process = self.process
        if process is None:
            return
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    def _close_lidar(self):
        lidar, self.lidar = self.lidar, None
        if lidar is None:
            return
        for method_name in ("stop", "stop_motor", "disconnect"):
            try:
                method = getattr(lidar, method_name, None)
                if method:
                    method()
            except Exception:
                pass

    def _run_mock(self, error=""):
        while self.running.is_set():
            now = time.time()
            obstacle_angle = 16.0 * math.sin(now * 0.55)
            points = []
            for angle in range(360):
                signed = self._signed_angle(angle)
                distance = 2200 + 280 * math.sin(math.radians(angle * 3))
                angular_error = abs(self._signed_angle(signed - obstacle_angle))
                if angular_error < 11:
                    distance = 760 + angular_error * 15 + 80 * math.sin(now * 0.8)
                elif 35 <= signed <= 63:
                    distance = 1350 + 90 * math.sin(math.radians(angle * 5))
                elif -75 <= signed <= -45:
                    distance = 1100 + 70 * math.cos(math.radians(angle * 4))
                points.append((angle, max(120, distance)))
            self._set_scan("mock", points, error=error)
            time.sleep(self.update_period)
