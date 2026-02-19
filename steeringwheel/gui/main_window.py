# gui/main_window.py

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QComboBox
)
from PyQt5.QtCore import QTimer

from gui.widgets.lidar_widget import LidarWidget
from gui.widgets.clickable_camera import ClickableCameraLabel

from gui.mixins.camera_mixin import CameraMixin
from gui.mixins.lane_mixin import LaneMixin
from gui.mixins.status_mixin import StatusMixin
from gui.mixins.lidar_mixin import LidarMixin


class ControlHubGUI( QWidget, CameraMixin, LaneMixin, StatusMixin, LidarMixin):

    def __init__(self, hub):
        super().__init__()
        self.hub = hub

        self.cur_speed = 0
        self.cur_steer = 90

        self.forward_pressed = False
        self.backward_pressed = False

        


        self.init_ui()
        self.init_timer()

    # =========================================================
    # UI SETUP
    # =========================================================

    def init_ui(self):

        self.setWindowTitle("AUTODRIVE – Control Station")
        self.resize(1200, 800)

        main = QVBoxLayout()

        # ================= MODE =================
        mode_row = QHBoxLayout()

        self.btn_manual = QPushButton("MANUAL")
        self.btn_follow = QPushButton("FOLLOW")

        self.btn_manual.clicked.connect(
            lambda: self.hub.set_mode("manual")
        )

        self.btn_follow.clicked.connect(
            lambda: self.hub.set_mode("follow")
        )

        mode_row.addWidget(self.btn_manual)
        mode_row.addWidget(self.btn_follow)

        # LEVEL buttons
        for i in range(2):
            btn = QPushButton(f"L{i}")
            btn.clicked.connect(
                lambda _, lv=i: self.hub.set_level(lv)
            )
            mode_row.addWidget(btn)

        main.addLayout(mode_row)

        # ===== LANE PANEL =====
        self.lane_group = QGroupBox("Lane Keeping Algorithm")
        lane_layout = QVBoxLayout()

        self.combo_algo = QComboBox()
        self.combo_algo.addItems(["geometry"])

        btn_row = QHBoxLayout()

        self.btn_lane_start = QPushButton("START")
        self.btn_lane_stop = QPushButton("STOP")
        self.btn_lane_extend = QPushButton("EXTEND VIEW")

        self.btn_lane_start.clicked.connect(self.start_lane_algo)
        self.btn_lane_stop.clicked.connect(self.stop_lane_algo)
        self.btn_lane_extend.clicked.connect(self.open_extend_view)

        btn_row.addWidget(self.btn_lane_start)
        btn_row.addWidget(self.btn_lane_stop)

        lane_layout.addWidget(QLabel("Algorithm:"))
        lane_layout.addWidget(self.combo_algo)
        lane_layout.addLayout(btn_row)
        lane_layout.addWidget(self.btn_lane_extend)

        self.lane_group.setLayout(lane_layout)
        main.addWidget(self.lane_group)


        # ================= DRIVE =================
        drive_group = QGroupBox("Drive Control")
        drive_layout = QVBoxLayout()

        btn_row = QHBoxLayout()

        self.btn_left = QPushButton("LEFT")
        self.btn_forward = QPushButton("FORWARD")
        self.btn_backward = QPushButton("BACKWARD")
        self.btn_right = QPushButton("RIGHT")
        self.btn_center = QPushButton("CENTER")

        btn_row.addWidget(self.btn_left)
        btn_row.addWidget(self.btn_forward)
        btn_row.addWidget(self.btn_backward)
        btn_row.addWidget(self.btn_right)
        btn_row.addWidget(self.btn_center)

        drive_layout.addLayout(btn_row)

        self.lbl_speed = QLabel("Speed: 0")
        self.lbl_steer = QLabel("Steer: 90")

        drive_layout.addWidget(self.lbl_speed)
        drive_layout.addWidget(self.lbl_steer)

        self.btn_emergency = QPushButton("EMERGENCY STOP")
        self.btn_emergency.setStyleSheet(
            "background-color: red; color: white; font-weight: bold; height: 40px;"
        )
        self.btn_emergency.clicked.connect(
            self.hub.emergency_stop
        )

        drive_layout.addWidget(self.btn_emergency)

        drive_group.setLayout(drive_layout)
        main.addWidget(drive_group)

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


        # ================= SENSOR =================
        self.sensor_group = QGroupBox("DOWN - FRONT - LIDAR")
        sensor_row = QHBoxLayout()

        self.lbl_cam_front = ClickableCameraLabel()
        self.lbl_cam_front.setFixedSize(320, 240)

        self.lbl_cam_down = ClickableCameraLabel()
        self.lbl_cam_down.setFixedSize(320, 240)

        self.lidar = LidarWidget()

        sensor_row.addWidget(self.lbl_cam_down)
        sensor_row.addWidget(self.lbl_cam_front)
        sensor_row.addWidget(self.lidar)

        self.sensor_group.setLayout(sensor_row)

        main.addWidget(self.sensor_group)



        # =================Done================
        self.setLayout(main)

        # ===== BUTTON BINDINGS =====

        self.btn_forward.pressed.connect(
            lambda: self._set_forward(True)
        )
        self.btn_forward.released.connect(
            lambda: self._set_forward(False)
        )

        self.btn_backward.pressed.connect(
            lambda: self._set_backward(True)
        )
        self.btn_backward.released.connect(
            lambda: self._set_backward(False)
        )

        self.btn_left.clicked.connect(
            lambda: self._set_steer(60)
        )
        self.btn_right.clicked.connect(
            lambda: self._set_steer(120)
        )
        self.btn_center.clicked.connect(
            lambda: self._set_steer(90)
        )

    # =========================================================
    # INPUT HANDLING
    # =========================================================

    def _set_forward(self, state):
        self.forward_pressed = state

    def _set_backward(self, state):
        self.backward_pressed = state

    def _set_steer(self, value):
        self.cur_steer = value
        self.hub.set_input(self.cur_speed, self.cur_steer)
        self.lbl_steer.setText(f"Steer: {value}")

    # =========================================================
    # INPUT TIMER
    # =========================================================

    def init_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_input)
        self.timer.start(50)
        self.lidar_timer = QTimer()
        self.lidar_timer.timeout.connect(self.update_lidar_view)
        self.lidar_timer.start(100)
        self.cam_timer = QTimer()
        self.cam_timer.timeout.connect(self.update_live_cameras)
        self.cam_timer.start(30)


    def _update_input(self):

        if self.forward_pressed:
            self.cur_speed = min(self.cur_speed + 5, 200)

        elif self.backward_pressed:
            self.cur_speed = max(self.cur_speed - 5, -200)

        else:
            if self.cur_speed > 0:
                self.cur_speed -= 10
            elif self.cur_speed < 0:
                self.cur_speed += 10

        self.hub.set_input(self.cur_speed, self.cur_steer)

        self.lbl_speed.setText(f"Speed: {self.cur_speed}")





