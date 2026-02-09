import time
import threading
import sys
import select
import math

from protocol import build_packet, CMD_DRIVE, CMD_PING
from uart_driver import UARTDriver

# ---- lane keeping imports (NEW) ----
from autodrive.lane_algo_state import LaneAlgoState
from autodrive.lane_algo_geometry import LaneAlgoGeometry

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

        self.uart = UARTDriver()

        # quick link check
        self.uart.send(build_packet(CMD_PING))
        time.sleep(0.1)

        # ====== MANUAL STATE (NEW – nhưng an toàn) ======
        self.manual_steer = STEER_CENTER
        self.manual_speed = 0

        # ====== LANE KEEPING (NEW) ======
        self.lane_state = LaneAlgoState()
        self.lane_algo = None
        self.lane_algo_running = False

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
        if self.level == 1 and self.lane_algo_running:
            # demo lane giả để test (thay bằng OpenCV sau)
            t = time.time()

            self.lane_algo.update()

            error = self.lane_state.error
            steer = STEER_CENTER + error * STEER_GAIN_L1

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
