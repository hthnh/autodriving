import time
import threading
import sys
import select

from protocol import build_packet, CMD_DRIVE, CMD_PING
from uart_driver import UARTDriver
from core.camera_manager import CameraManager


# ---- lane keeping imports (NEW) ----
from algorithms.lane_algo_state import LaneAlgoState
from algorithms.lane_algo_geometry import LaneAlgoGeometry


from algorithms.follow_state import FollowState
from algorithms.target_tracker import TargetTracker
from algorithms.lidar_algo import LidarProcessor
from core.behavior import follow_arbitration


# ---------------- CONFIG ----------------
STEER_CENTER = 90
MAX_SPEED = 200
SEND_PERIOD = 0.05   # 20 Hz

STEER_GAIN_L1 = 25   # gain cho lane keeping level 1
# ----------------------------------------


def clamp(val, min_v, max_v):
    return max(min_v, min(max_v, val))


class ControlHub:
    def __init__(self):
        # ====== CŨ ======
        self.level = 0                      # 0–5
        self.active_source = "manual"       # manual | autodrive
        self.last_cmd = None
        self.last_send = 0.0



        # ====== MANUAL STATE (NEW – nhưng an toàn) ======
        self.manual_steer = STEER_CENTER
        self.manual_speed = 0

        # ====== LANE KEEPING (NEW) ======

        # ===== FOLLOW SYSTEM =====


        # ===== INTERNAL CONTROL LOOP =====
        self.running = True
        self.control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True
        )
        self.control_thread.start()


    def _control_loop(self):
        while self.running:
            # gửi theo trạng thái hiện tại
            if self.active_source == "manual":
                self._send_drive(
                    self.manual_steer,
                    self.manual_speed,
                    "manual"
                )
            else:
                # autodrive vẫn dùng manual làm base,
                # vì merge logic nằm trong _send_drive
                self._send_drive(
                    self.manual_steer,
                    self.manual_speed,
                    "autodrive"
                )

            time.sleep(SEND_PERIOD)


    def init_follow_tracker(self, bbox):
        frame = self.camera_manager.get_frame(1)  # front cam

        if frame is None:
            print("No frame for tracker init")
            return False

        self.target_tracker.init_tracker(frame, bbox)
        print("Tracker initialized:", bbox)
        return True




    def start_follow(self):
        self.follow_running = True

    def stop_follow(self):
        self.follow_running = False

    # ---------- AUTHORITY RULE ----------
    def can_accept(self, source: str) -> bool:
        if self.level == 0:
            return source == "manual"
        elif self.level in (1, 2):
            return source in ("manual", "autodrive")
        else:   # level 3–5
            return source == "autodrive"

    # ---------- PUBLIC API ----------
    def send_manual(self, steer, speed):
        # giữ lại trạng thái manual
        self.manual_steer = steer
        self.manual_speed = speed

        if self.can_accept("manual"):
            self.active_source = "manual"
            self._send_drive(steer, speed, "manual")

    def send_autodrive(self, steer, speed):
        if self.can_accept("autodrive"):
            self.active_source = "autodrive"
            self._send_drive(steer, speed, "autodrive")

    def set_level(self, level: int):
        self.level = clamp(level, 0, 5)

    # ---------- LANE ALGORITHM CONTROL (NEW) ----------
    def select_lane_algo(self, name: str):
        if self.lane_algo_running:
            return False

        if name == "geometry":
            self.lane_algo = LaneAlgoGeometry(
            state=self.lane_state,
            camera_manager=self.camera_manager,
            cam_id=0,   # down camera
            img_width=640,
            car_center_offset_px=0
        )

        else:
            self.lane_algo = None

        return True

    def start_lane_algo(self):
        if self.lane_algo is None:
            return False
        self.lane_algo_running = True
        self.lane_algo.start()
        return True

    def stop_lane_algo(self):
        if not self.lane_algo_running:
            return
        self.lane_algo_running = False
        self.lane_algo.stop()

    # ---------- LOW LEVEL ----------
    def _send_drive(self, steer, speed, source):
        now = time.time()
        if now - self.last_send < SEND_PERIOD:
            return

        self.last_send = now

        # ====== LEVEL 1 MERGE (NEW, NHƯNG ĐÚNG CHỖ) ======
        if self.lane_algo_running and not self.follow_running:

            # demo lane giả để test (thay bằng OpenCV sau)
            t = time.time()

            self.lane_algo.update()

            error = self.lane_state.error
            kp = self.runtime_params.get("kp_steering", STEER_GAIN_L1)
            steer = STEER_CENTER + error * kp * 100



        if self.follow_running:
            self.lidar_processor.stop_threshold = self.runtime_params["stop_threshold"]
            frame = self.camera_manager.get_frame(1)
            self.target_tracker.update(frame)

            # mock lidar for now
            scan = [(a, 2.0) for a in range(360)]
            lidar_result = self.lidar_processor.process(scan)

            steer, speed = follow_arbitration(
                self.follow_state,
                lidar_result,
                self.runtime_params
            )
        



        # ====== CŨ ======
        steer = clamp(int(steer), 45, 135)
        speed = clamp(int(speed), -MAX_SPEED, MAX_SPEED)

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

        self.last_cmd = {
            "steer": steer,
            "speed": speed,
            "source": source,
            "time": now
        }

    def stop(self):
        self.uart.send(build_packet(
            CMD_DRIVE,
            bytes([STEER_CENTER, 0, 0, 0, 0])
        ))
        self.uart.close()
        self.running = False



    def get_mock_scan(self):
        scan = []
        for angle in range(360):
            dist = 2.0

            # obstacle trước mặt
            if -10 < ((angle + 180) % 360 - 180) < 10:
                dist = 0.4

            scan.append((angle, dist))

        return scan

