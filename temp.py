
#main_window.py
import numpy as np
import math
import time


from PyQt5.QtGui import QImage, QPixmap, QColor, QPen

from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton,
    QSlider, QComboBox,
    QGroupBox, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer

from control_hub import ControlHub

from algorithms.lane_algo_state import LaneAlgoState

from gui.widgets.lane_extend_window import LaneExtendWindow

from gui.widgets.clickable_camera import ClickableCameraLabel

from gui.widgets.lidar_widget import LidarWidget

from gui.mixins.camera_mixin import CameraMixin
from gui.mixins.lane_mixin import LaneMixin
from gui.mixins.status_mixin import StatusMixin
from gui.mixins.lidar_mixin import LidarMixin


# ================= MAIN GUI =================

class ControlHubGUI( QWidget, CameraMixin, LaneMixin, StatusMixin, LidarMixin):

    def __init__(self, hub: ControlHub):
        super().__init__()
        self.hub = hub

        self.mode = "manual"   # manual | auto | follow

        self.level = None

        self.cur_speed = 0
        self.cur_steer = 90

        self.extend_window = None
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_live_cameras)
        self.gui_timer.start(30)   # 30 FPS GUI


        self.init_ui()
        self.init_timer()
        self.apply_ui_rules()

    # ================= UI =================

    def init_ui(self):
        self.setWindowTitle("Autodrive Control Hub")
        self.setFixedSize(1000, 1200)

        main = QVBoxLayout()

        # ===== MODE =====
        mode_box = QHBoxLayout()
        self.btn_manual = QPushButton("MANUAL")
        self.btn_auto = QPushButton("AUTO")
        self.btn_follow = QPushButton("FOLLOW")

        self.btn_manual.clicked.connect(lambda: self.set_mode("manual"))
        self.btn_auto.clicked.connect(lambda: self.set_mode("auto"))
        self.btn_follow.clicked.connect(lambda: self.set_mode("follow"))    

        mode_box.addWidget(self.btn_manual)
        mode_box.addWidget(self.btn_auto)
        mode_box.addWidget(self.btn_follow)
        main.addLayout(mode_box)

        # ===== CAMERA CONTROL =====
        self.cam_group = QGroupBox("Camera Control")
        cam_layout = QHBoxLayout()

        self.btn_front_cam = QPushButton("Front Cam ON")
        self.btn_front_cam.clicked.connect(self.toggle_front_cam)

        self.btn_down_cam = QPushButton("Down Cam ON")
        self.btn_down_cam.clicked.connect(self.toggle_down_cam)

        cam_layout.addWidget(self.btn_front_cam)
        cam_layout.addWidget(self.btn_down_cam)

        self.cam_group.setLayout(cam_layout)
        main.addWidget(self.cam_group)




        # ===== SENSOR VIEW =====
        sensor_layout = QHBoxLayout()

        # Camera View


        self.lbl_cam_front = ClickableCameraLabel()
        self.lbl_cam_front.setText("FRONT CAMERA")
        self.lbl_cam_front.callback = self.on_target_selected
        self.lbl_cam_front.setFixedSize(320, 240)
        self.lbl_cam_front.setStyleSheet("background: black;")





        self.lbl_cam_down = QLabel("Down Camera")
        self.lbl_cam_down.setFixedSize(320, 240)
        self.lbl_cam_down.setStyleSheet("background: black;")

        # Lidar View
        self.lidar_widget = LidarWidget()

        sensor_layout.addWidget(self.lbl_cam_front)
        sensor_layout.addWidget(self.lbl_cam_down)
        sensor_layout.addWidget(self.lidar_widget)

        main.addLayout(sensor_layout)





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



        # ===== PARAM PANEL =====
        self.param_group = QGroupBox("Runtime Parameters (Debug)")
        param_layout = QVBoxLayout()

        # Stop threshold
        self.lbl_stop = QLabel("Stop Threshold: 0.50")
        self.slider_stop = QSlider(Qt.Horizontal)
        self.slider_stop.setRange(10, 200)   # 0.10 – 2.00
        self.slider_stop.setValue(50)
        self.slider_stop.valueChanged.connect(self.on_stop_change)

        param_layout.addWidget(self.lbl_stop)
        param_layout.addWidget(self.slider_stop)

        # Steering Kp
        self.lbl_kp = QLabel("Steering Kp: 0.004")
        self.slider_kp = QSlider(Qt.Horizontal)
        self.slider_kp.setRange(1, 50)
        self.slider_kp.setValue(4)
        self.slider_kp.valueChanged.connect(self.on_kp_change)

        param_layout.addWidget(self.lbl_kp)
        param_layout.addWidget(self.slider_kp)

        self.param_group.setLayout(param_layout)
        main.addWidget(self.param_group)




        # ===== STATUS =====
        self.lbl_status = QLabel("")
        main.addWidget(self.lbl_status)
        self.lbl_follow = QLabel("Follow: visible=False error=0.00 steer=90")
        main.addWidget(self.lbl_follow)


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

        elif self.mode == "follow":
            # tạm thời giữ speed manual
            self.hub.send_manual(self.cur_steer, self.cur_speed)
        

        self.update_lidar_view()
        self.update_follow_debug()




    # ================= STATUS =================

    def on_stop_change(self, v):
        val = v / 100.0
        self.lbl_stop.setText(f"Stop Threshold: {val:.2f}")
        self.hub.runtime_params["stop_threshold"] = val

    def on_kp_change(self, v):
        val = v / 1000.0
        self.lbl_kp.setText(f"Steering Kp: {val:.3f}")
        self.hub.runtime_params["kp_steering"] = val


