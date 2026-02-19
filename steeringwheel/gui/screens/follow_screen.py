# gui/screen/follow_screen.py
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt

from gui.widgets.clickable_camera import ClickableCameraLabel
from gui.widgets.lidar_widget import LidarWidget


class FollowScreen(QWidget):

    def __init__(self, hub):
        super().__init__()
        self.hub = hub

        self.lidar_enabled = False
        self.follow_enabled = False

        self.init_ui()
        self.init_timers()

    # ==================================================
    # UI
    # ==================================================

    def init_ui(self):

        main = QVBoxLayout()

        # ================= SENSOR VIEW =================
        sensor_row = QHBoxLayout()

        self.lbl_cam_front = ClickableCameraLabel()
        self.lbl_cam_front.setFixedSize(320, 240)
        self.lbl_cam_front.callback = self._on_target_selected

        self.lbl_cam_down = ClickableCameraLabel()
        self.lbl_cam_down.setFixedSize(320, 240)

        self.lidar_widget = LidarWidget()

        sensor_row.addWidget(self.lbl_cam_front)
        sensor_row.addWidget(self.lbl_cam_down)
        sensor_row.addWidget(self.lidar_widget)

        main.addLayout(sensor_row)

        # ================= STATUS =================
        status_row = QHBoxLayout()

        self.lbl_speed = QLabel("Speed: 0")
        self.lbl_steer = QLabel("Steer: 90")
        self.lbl_error = QLabel("Error: 0.0")

        status_row.addWidget(self.lbl_speed)
        status_row.addWidget(self.lbl_steer)
        status_row.addWidget(self.lbl_error)

        main.addLayout(status_row)

        # ================= CONTROL BUTTONS =================
        btn_row = QHBoxLayout()

        self.btn_front_cam = QPushButton("Front Cam ON")
        self.btn_down_cam = QPushButton("Down Cam ON")
        self.btn_lidar = QPushButton("Lidar ON")
        self.btn_start_follow = QPushButton("START FOLLOW")

        btn_row.addWidget(self.btn_front_cam)
        btn_row.addWidget(self.btn_down_cam)
        btn_row.addWidget(self.btn_lidar)
        btn_row.addWidget(self.btn_start_follow)

        main.addLayout(btn_row)

        # ================= EMERGENCY =================
        self.btn_emergency = QPushButton("EMERGENCY STOP")
        self.btn_emergency.setStyleSheet(
            "background-color: red; color: white; font-weight: bold; height: 40px;"
        )
        main.addWidget(self.btn_emergency)

        self.setLayout(main)

        # ================= BIND =================

        self.btn_front_cam.clicked.connect(self._toggle_front_cam)
        self.btn_down_cam.clicked.connect(self._toggle_down_cam)
        self.btn_lidar.clicked.connect(self._toggle_lidar)
        self.btn_start_follow.clicked.connect(self._toggle_follow)

        self.btn_emergency.clicked.connect(self.hub.emergency_stop)

    # ==================================================
    # CAMERA
    # ==================================================

    def _toggle_front_cam(self):
        if self.hub.is_camera_running(1):
            self.hub.stop_camera(1)
            self.btn_front_cam.setText("Front Cam ON")
        else:
            self.hub.start_camera(1)
            self.btn_front_cam.setText("Front Cam OFF")

    def _toggle_down_cam(self):
        if self.hub.is_camera_running(0):
            self.hub.stop_camera(0)
            self.btn_down_cam.setText("Down Cam ON")
        else:
            self.hub.start_camera(0)
            self.btn_down_cam.setText("Down Cam OFF")

    # ==================================================
    # LIDAR (placeholder)
    # ==================================================

    def _toggle_lidar(self):
        self.lidar_enabled = not self.lidar_enabled

        if self.lidar_enabled:
            self.btn_lidar.setText("Lidar OFF")
        else:
            self.btn_lidar.setText("Lidar ON")

        # Sau này:
        # self.hub.start_lidar()
        # self.hub.stop_lidar()

    # ==================================================
    # FOLLOW (placeholder)
    # ==================================================

    def _toggle_follow(self):

        self.follow_enabled = not self.follow_enabled

        if self.follow_enabled:
            self.btn_start_follow.setText("STOP FOLLOW")
            self.hub.set_mode("follow")
        else:
            self.btn_start_follow.setText("START FOLLOW")
            self.hub.set_mode("manual")

    # ==================================================
    # TARGET SELECT
    # ==================================================

    def _on_target_selected(self, bbox):

        frame = self.hub.get_camera_frame(1)
        if frame is None:
            return

        fh, fw, _ = frame.shape
        lw = self.lbl_cam_front.width()
        lh = self.lbl_cam_front.height()

        scale_x = fw / lw
        scale_y = fh / lh

        x, y, w, h = bbox

        scaled_bbox = (
            int(x * scale_x),
            int(y * scale_y),
            int(w * scale_x),
            int(h * scale_y)
        )

        self.hub.init_follow_target(scaled_bbox)

    # ==================================================
    # TIMERS
    # ==================================================

    def init_timers(self):

        self.cam_timer = QTimer()
        self.cam_timer.timeout.connect(self._update_cameras)
        self.cam_timer.start(30)

        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_status)
        self.status_timer.start(100)

    def _update_cameras(self):

        frame_front = self.hub.get_camera_frame(1)
        if frame_front is not None:
            self._show_frame(frame_front, self.lbl_cam_front)

        frame_down = self.hub.get_camera_frame(0)
        if frame_down is not None:
            self._show_frame(frame_down, self.lbl_cam_down)

        if self.lidar_enabled:
            scan = self.hub.get_mock_scan()
            self.lidar_widget.update_data(scan, None)

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

    def _update_status(self):

        status = self.hub.get_vehicle_status()

        self.lbl_speed.setText(f"Speed: {status['speed']}")
        self.lbl_steer.setText(f"Steer: {status['steer']}")


        follow = self.hub.get_follow_status()
        self.lbl_error.setText(f"Error: {follow['error']:.3f}")
