#gui/screens/autodrive_screen.py
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt

from gui.widgets.clickable_camera import ClickableCameraLabel
from gui.widgets.lidar_widget import LidarWidget

from .modules.steering_module import SteeringModule
from .modules.speed_module import SpeedModule
from .modules.avoid_module import AvoidModule


class AutoDriveScreen(QWidget):

    def __init__(self, hub):
        super().__init__()
        self.hub = hub
        self.level = 0

        self.init_ui()
        self.init_timers()

    # ==================================================
    # UI
    # ==================================================

    def init_ui(self):

        main = QVBoxLayout()

        # ================= SENSOR =================
        sensor_row = QHBoxLayout()

        self.cam_front = ClickableCameraLabel()
        self.cam_front.setFixedSize(320, 240)

        self.cam_down = ClickableCameraLabel()
        self.cam_down.setFixedSize(320, 240)

        self.lidar_widget = LidarWidget()

        sensor_row.addWidget(self.cam_front)
        sensor_row.addWidget(self.cam_down)
        sensor_row.addWidget(self.lidar_widget)

        main.addLayout(sensor_row)

        # ================= MODULES =================
        module_row = QHBoxLayout()

        self.steering_module = SteeringModule(self.hub)
        self.speed_module = SpeedModule(self.hub)
        self.avoid_module = AvoidModule(self.hub)

        module_row.addWidget(self.steering_module)
        module_row.addWidget(self.speed_module)
        module_row.addWidget(self.avoid_module)

        main.addLayout(module_row)

        # ================= CONTROL =================
        control_row = QHBoxLayout()

        self.lbl_level = QLabel("Level: 0")

        self.btn_lv0 = QPushButton("L0")
        self.btn_lv1 = QPushButton("L1")
        self.btn_lv2 = QPushButton("L2")

        self.btn_emergency = QPushButton("EMERGENCY")
        self.btn_emergency.setStyleSheet("background-color: red; color: white;")

        control_row.addWidget(self.lbl_level)
        control_row.addWidget(self.btn_lv0)
        control_row.addWidget(self.btn_lv1)
        control_row.addWidget(self.btn_lv2)
        control_row.addWidget(self.btn_emergency)

        main.addLayout(control_row)

        self.setLayout(main)

        # ================= BIND =================
        self.btn_lv0.clicked.connect(lambda: self._set_level(0))
        self.btn_lv1.clicked.connect(lambda: self._set_level(1))
        self.btn_lv2.clicked.connect(lambda: self._set_level(2))

        self.btn_emergency.clicked.connect(self.hub.emergency_stop)

    # ==================================================
    # LEVEL CONTROL
    # ==================================================

    def _set_level(self, level):

        self.level = level
        self.hub.set_mode("auto")
        self.hub.set_level(level)

        self.lbl_level.setText(f"Level: {level}")

        self.steering_module.set_level(level)
        self.speed_module.set_level(level)
        self.avoid_module.set_level(level)

    # ==================================================
    # TIMERS
    # ==================================================

    def init_timers(self):

        self.timer = QTimer()
        self.timer.timeout.connect(self._update_cameras)
        self.timer.start(30)

    def _update_cameras(self):

        frame_front = self.hub.get_camera_frame(1)
        if frame_front is not None:
            self._show_frame(frame_front, self.cam_front)

        frame_down = self.hub.get_camera_frame(0)
        if frame_down is not None:
            self._show_frame(frame_down, self.cam_down)

    def _show_frame(self, frame, label):

        h, w, ch = frame.shape
        bytes_per_line = ch * w

        qimg = QImage(
            frame.data, w, h,
            bytes_per_line,
            QImage.Format_RGB888
        )

        pix = QPixmap.fromImage(qimg).scaled(
            label.width(),
            label.height(),
            Qt.KeepAspectRatio
        )

        label.setPixmap(pix)
