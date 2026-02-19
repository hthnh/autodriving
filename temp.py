import time
import threading

from protocol import build_packet, CMD_DRIVE, CMD_PING
from uart_driver import UARTDriver

from core.camera_manager import CameraManager

from algorithms.lane_algo_state import LaneAlgoState
from algorithms.lane_algo_geometry import LaneAlgoGeometry
from algorithms.follow_state import FollowState
from algorithms.target_tracker import TargetTracker
from algorithms.lidar_algo import LidarProcessor

from control.modes.manual_mode import ManualMode
from control.modes.follow_mode import FollowMode
from control.modes.autodrive.level0 import Level0Mode
from control.modes.autodrive.level1 import Level1Mode


STEER_CENTER = 90
MAX_SPEED = 200
SEND_PERIOD = 0.05


def clamp(val, min_v, max_v):
    return max(min_v, min(max_v, val))


class ControlHub:

    def __init__(self):

        # ================= INPUT STATE =================
        self.input_state = {
            "speed": 0,
            "steer": STEER_CENTER,
            "mode": "manual",
            "level": 0
        }

        self.runtime_params = {
            "stop_threshold": 0.5,
            "kp_steering": 0.004
        }

        self.emergency = False

        # ==== NEW: last command tracking ====
        self.last_speed = 0
        self.last_steer = STEER_CENTER

        # ================= DEVICES =================
        self.uart = UARTDriver()
        self.uart.send(build_packet(CMD_PING))
        time.sleep(0.1)

        self.camera_manager = CameraManager()
        self.camera_manager.add_camera(1, {"size": (640, 480), "fps": 30})
        self.camera_manager.add_camera(0, {"size": (640, 480), "fps": 30})

        # ================= ALGORITHMS =================
        self.lane_state = LaneAlgoState()
        self.lane_algo = None
        self.lane_algo_running = False

        self.follow_state = FollowState()
        self.target_tracker = TargetTracker(self.follow_state)

        self.lidar_processor = LidarProcessor(
            front_width=60,
            side_width=60,
            stop_threshold=self.runtime_params["stop_threshold"]
        )

        # ================= MODE =================
        self.mode = ManualMode(self)

        # ================= LOOP =================
        self.running = True
        self.thread = threading.Thread(
            target=self._loop,
            daemon=True
        )
        self.thread.start()

    # =================================================
    # ================= CONTROL API ===================
    # =================================================

    def set_input(self, speed, steer):
        self.input_state["speed"] = speed
        self.input_state["steer"] = steer

    def set_mode(self, mode_name):
        self.input_state["mode"] = mode_name

        if mode_name == "manual":
            self.mode = ManualMode(self)

        elif mode_name == "follow":
            self.mode = FollowMode(self)

        elif mode_name == "auto":
            # giữ mode, level sẽ quyết định
            pass

    def set_level(self, level):
        self.input_state["level"] = level

        if level == 0:
            self.mode = Level0Mode(self)

        elif level == 1:
            self.mode = Level1Mode(self)

    def emergency_stop(self):
        self.emergency = True

    def clear_emergency(self):
        self.emergency = False

    # =================================================
    # ================= STATUS API ====================
    # =================================================

    def get_vehicle_status(self):
        return {
            "mode": self.input_state["mode"],
            "level": self.input_state["level"],
            "speed": self.last_speed,
            "steer": self.last_steer,
            "emergency": self.emergency
        }

    # =================================================
    # ================= CAMERA API ====================
    # =================================================

    def start_camera(self, cam_id):
        self.camera_manager.start(cam_id)

    def stop_camera(self, cam_id):
        self.camera_manager.stop(cam_id)

    def is_camera_running(self, cam_id):
        return self.camera_manager.is_running(cam_id)

    def get_camera_frame(self, cam_id):
        return self.camera_manager.get_frame(cam_id)

    # =================================================
    # ================= FOLLOW API ====================
    # =================================================

    def init_follow_target(self, bbox):
        frame = self.get_camera_frame(1)
        if frame is None:
            return False
        self.target_tracker.init_tracker(frame, bbox)
        return True

    def get_follow_status(self):
        return {
            "visible": self.follow_state.target_visible,
            "error": self.follow_state.target_error,
            "steer": self.follow_state.final_steer,
            "speed": self.follow_state.final_speed
        }

    # =================================================
    # ================= LANE API ======================
    # =================================================

    def get_lane_status(self):
        return {
            "running": self.lane_algo_running,
            "error": self.lane_state.error,
            "confidence": getattr(self.lane_state, "confidence", 0.0)
        }

    def get_lane_debug_frames(self):
        return getattr(self.lane_state, "debug_frames", {})

    # =================================================
    # ================= LOOP ==========================
    # =================================================

    def _loop(self):

        while self.running:

            if self.emergency:
                steer = STEER_CENTER
                speed = 0
            else:
                steer, speed = self.mode.process(self.input_state)

            steer = clamp(int(steer), 45, 135)
            speed = clamp(int(speed), -MAX_SPEED, MAX_SPEED)

            # ==== NEW: store last command ====
            self.last_steer = steer
            self.last_speed = speed

            self._send_drive(steer, speed)

            time.sleep(SEND_PERIOD)

    # =================================================
    # ================= DRIVE LAYER ===================
    # =================================================

    def _send_drive(self, steer, speed):

        if speed >= 0:
            dir_l = dir_r = 1
            spd_l = spd_r = speed
        else:
            dir_l = dir_r = 2
            spd_l = spd_r = abs(speed)

        payload = bytes([
            steer,
            dir_l, spd_l,
            dir_r, spd_r
        ])

        self.uart.send(build_packet(CMD_DRIVE, payload))
