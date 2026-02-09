import sys
import cv2
import numpy as np

from PyQt5.QtGui import QImage, QPixmap

from PyQt5.QtWidgets import (
    QApplication, QWidget,
    QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton,
    QSlider, QComboBox,
    QGroupBox, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer

from control_hub import ControlHub

from autodrive.lane_algo_state import LaneAlgoState


# ================= EXTENDED WINDOW (DUMMY) =================
class LaneExtendWindow(QWidget):
    def __init__(self, hub):
        super().__init__()
        self.hub = hub

        self.setWindowTitle("Lane Algorithm – Extended View")
        self.setFixedSize(700, 520)

        layout = QVBoxLayout()

        # ===== STATUS =====
        self.lbl_error = QLabel("Error: 0.000")
        self.lbl_conf = QLabel("Confidence: 0.00")

        layout.addWidget(self.lbl_error)
        layout.addWidget(self.lbl_conf)

        # ===== IMAGE VIEWS =====
        img_layout = QHBoxLayout()

        self.lbl_raw = QLabel("RAW")
        self.lbl_raw.setFixedSize(320, 240)
        self.lbl_raw.setStyleSheet("background: black;")

        self.lbl_edges = QLabel("EDGES")
        self.lbl_edges.setFixedSize(320, 240)
        self.lbl_edges.setStyleSheet("background: black;")

        img_layout.addWidget(self.lbl_raw)
        img_layout.addWidget(self.lbl_edges)

        layout.addLayout(img_layout)

        self.setLayout(layout)

        # ===== TIMER =====
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_view)
        self.timer.start(100)   # 10 FPS UI

    # ================= UPDATE =================

    def refresh_view(self):
        state = self.hub.lane_state

        # --- text ---
        self.lbl_error.setText(f"Error: {state.error:.3f}")
        self.lbl_conf.setText(f"Confidence: {state.confidence:.2f}")

        if not hasattr(state, "debug_frames"):
            return

        frames = state.debug_frames

        # --- RAW IMAGE ---
        if "raw" in frames:
            self._show_rgb(frames["raw"], self.lbl_raw)

        # --- EDGES IMAGE ---
        if "edges" in frames:
            self._show_gray(frames["edges"], self.lbl_edges)

    # ================= HELPERS =================

    def _show_rgb(self, img, label):
        # img: RGB888 numpy array
        h, w, ch = img.shape
        bytes_per_line = ch * w
        qimg = QImage(
            img.data, w, h,
            bytes_per_line,
            QImage.Format_RGB888
        )
        pix = QPixmap.fromImage(qimg).scaled(
            label.width(),
            label.height(),
            Qt.KeepAspectRatio
        )
        label.setPixmap(pix)

    def _show_gray(self, img, label):
        # img: grayscale numpy array
        h, w = img.shape
        qimg = QImage(
            img.data, w, h,
            w,
            QImage.Format_Grayscale8
        )
        pix = QPixmap.fromImage(qimg).scaled(
            label.width(),
            label.height(),
            Qt.KeepAspectRatio
        )
        label.setPixmap(pix)





# ================= MAIN GUI =================

class ControlHubGUI(QWidget):
    def __init__(self, hub: ControlHub):
        super().__init__()
        self.hub = hub

        self.mode = "manual"   # manual | auto
        self.level = None

        self.cur_speed = 0
        self.cur_steer = 90

        self.extend_window = None

        self.init_ui()
        self.init_timer()
        self.apply_ui_rules()

    # ================= UI =================

    def init_ui(self):
        self.setWindowTitle("Autodrive Control Hub")
        self.setFixedSize(520, 580)

        main = QVBoxLayout()

        # ===== MODE =====
        mode_box = QHBoxLayout()
        self.btn_manual = QPushButton("MANUAL")
        self.btn_auto = QPushButton("AUTO")

        self.btn_manual.clicked.connect(lambda: self.set_mode("manual"))
        self.btn_auto.clicked.connect(lambda: self.set_mode("auto"))

        mode_box.addWidget(self.btn_manual)
        mode_box.addWidget(self.btn_auto)
        main.addLayout(mode_box)

        # ===== LEVEL =====
        level_group = QGroupBox("Autonomy Level")
        level_layout = QHBoxLayout()

        self.level_buttons = {}
        for i in range(6):
            btn = QPushButton(f"L{i}")
            btn.clicked.connect(lambda _, lv=i: self.set_level(lv))
            self.level_buttons[i] = btn
            level_layout.addWidget(btn)

        level_group.setLayout(level_layout)
        main.addWidget(level_group)

        # ===== SPEED =====
        self.lbl_speed = QLabel("Speed: 0")
        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(-200, 200)
        self.slider_speed.valueChanged.connect(self.on_speed_change)

        main.addWidget(self.lbl_speed)
        main.addWidget(self.slider_speed)

        # ===== STEER =====
        self.lbl_steer = QLabel("Steering: 90")
        self.slider_steer = QSlider(Qt.Horizontal)
        self.slider_steer.setRange(45, 135)
        self.slider_steer.setValue(90)
        self.slider_steer.valueChanged.connect(self.on_steer_change)

        main.addWidget(self.lbl_steer)
        main.addWidget(self.slider_steer)

        # ===== LANE CONTAINER (L1) =====
        self.lane_group = QGroupBox("Lane Keeping Algorithm")
        lane_layout = QVBoxLayout()

        self.combo_algo = QComboBox()
        # map đúng với ControlHub.select_lane_algo
        self.combo_algo.addItems(["geometry"])

        algo_btns = QHBoxLayout()
        self.btn_algo_start = QPushButton("START")
        self.btn_algo_stop = QPushButton("STOP")

        self.btn_algo_start.clicked.connect(self.start_lane_algo)
        self.btn_algo_stop.clicked.connect(self.stop_lane_algo)

        algo_btns.addWidget(self.btn_algo_start)
        algo_btns.addWidget(self.btn_algo_stop)

        self.btn_extend = QPushButton("EXTEND VIEW")
        self.btn_extend.clicked.connect(self.open_extend_view)

        lane_layout.addWidget(QLabel("Algorithm:"))
        lane_layout.addWidget(self.combo_algo)
        lane_layout.addLayout(algo_btns)
        lane_layout.addWidget(self.btn_extend)

        self.lane_group.setLayout(lane_layout)
        main.addWidget(self.lane_group)

        # ===== STATUS =====
        self.lbl_status = QLabel("")
        main.addWidget(self.lbl_status)

        # ===== EMERGENCY STOP =====
        self.btn_emergency = QPushButton("EMERGENCY STOP")
        self.btn_emergency.setStyleSheet(
            "background-color: red; color: white; font-weight: bold; height: 40px;"
        )
        self.btn_emergency.clicked.connect(self.emergency_stop)
        main.addWidget(self.btn_emergency)

        self.setLayout(main)

    # ================= MODE / LEVEL =================

    def emergency_stop(self):
        self.cur_speed = 0
        self.cur_steer = 90
        self.slider_speed.setValue(0)
        self.slider_steer.setValue(90)
        self.hub.send_manual(90, 0)

    def set_mode(self, mode):
        self.mode = mode
        self.level = None
        self.apply_ui_rules()

    def set_level(self, level):
        if self.mode != "auto":
            return

        self.level = level
        self.hub.set_level(level)
        self.apply_ui_rules()

        if level >= 2:
            QMessageBox.information(
                self, "Info",
                f"Level {level} not implemented yet."
            )

    # ================= UI RULE ENGINE =================

    def apply_ui_rules(self):
        is_manual = self.mode == "manual"
        is_auto = self.mode == "auto"

        for btn in self.level_buttons.values():
            btn.setEnabled(is_auto)

        self.slider_speed.setEnabled(False)
        self.slider_steer.setEnabled(False)
        self.lane_group.setVisible(False)

        if is_manual:
            self.slider_speed.setEnabled(True)
            self.slider_steer.setEnabled(True)
            self.hub.set_level(0)
            self.hub.active_source = "manual"

        if is_auto and self.level is not None:
            self.hub.active_source = "autodrive"

            if self.level == 0:
                self.slider_speed.setEnabled(True)
                self.slider_steer.setEnabled(True)

            elif self.level == 1:
                self.slider_speed.setEnabled(True)
                self.slider_steer.setEnabled(False)
                self.lane_group.setVisible(True)

        self.update_status()

    # ================= INPUT =================

    def on_speed_change(self, v):
        self.cur_speed = v
        self.lbl_speed.setText(f"Speed: {v}")

    def on_steer_change(self, v):
        self.cur_steer = v
        self.lbl_steer.setText(f"Steering: {v}")

    # ================= LANE ALGO CONTROL =================

    def start_lane_algo(self):
        algo = self.combo_algo.currentText()
        ok = self.hub.select_lane_algo(algo)
        if not ok:
            QMessageBox.warning(self, "Warning", "Lane algorithm already running")
            return
        self.hub.start_lane_algo()

    def stop_lane_algo(self):
        self.hub.stop_lane_algo()

    # ================= EXTEND =================

    def open_extend_view(self):
        if self.extend_window is None:
            self.extend_window = LaneExtendWindow(self.hub)
        self.extend_window.show()

    # ================= CONTROL LOOP =================

    def init_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.control_tick)
        self.timer.start(50)

    def control_tick(self):
        if self.mode == "manual":
            self.hub.send_manual(self.cur_steer, self.cur_speed)

        elif self.mode == "auto":
            if self.level == 0:
                self.hub.send_manual(self.cur_steer, self.cur_speed)

            elif self.level == 1:
                # speed từ manual, steer do lane algo
                self.hub.send_manual(self.cur_steer, self.cur_speed)

    # ================= STATUS =================

    def update_status(self):
        txt = f"MODE: {self.mode.upper()}"
        if self.level is not None:
            txt += f" | LEVEL: L{self.level}"
        if self.hub.lane_algo_running:
            txt += " | LANE: RUNNING"
        txt += f" | error={self.hub.lane_state.error:.2f}"
        
        self.lbl_status.setText(txt)



# ================= MAIN =================

def main():
    hub = ControlHub()

    app = QApplication(sys.argv)
    gui = ControlHubGUI(hub)
    gui.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
