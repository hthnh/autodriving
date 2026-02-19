# gui/main_window.py

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QComboBox, QStackedWidget
)
from PyQt5.QtCore import QTimer

from gui.widgets.lidar_widget import LidarWidget
from gui.widgets.clickable_camera import ClickableCameraLabel

from gui.mixins.camera_mixin import CameraMixin
from gui.mixins.lane_mixin import LaneMixin
from gui.mixins.status_mixin import StatusMixin
from gui.mixins.lidar_mixin import LidarMixin

from gui.screens.manual_screen import ManualScreen
from gui.screens.follow_screen import FollowScreen
from gui.screens.autodrive_screen import AutoDriveScreen


class ControlHubGUI( QWidget, CameraMixin, LaneMixin, StatusMixin, LidarMixin):

    def __init__(self, hub):
        super().__init__()
        self.hub = hub

        self.cur_speed = 0
        self.cur_steer = 90

        self.forward_pressed = False
        self.backward_pressed = False

        


        self.init_ui()

    # =========================================================
    # UI SETUP
    # =========================================================
    def init_ui(self):

        self.setWindowTitle("Autodrive Control Hub")
        self.resize(1200, 700)

        main = QVBoxLayout()

        # ================= MODE SELECT =================
        mode_row = QHBoxLayout()

        self.btn_manual = QPushButton("MANUAL")
        self.btn_follow = QPushButton("FOLLOW")
        self.btn_auto = QPushButton("AUTO")

        mode_row.addWidget(self.btn_manual)
        mode_row.addWidget(self.btn_follow)
        mode_row.addWidget(self.btn_auto)

        main.addLayout(mode_row)

        # ================= STACK =================
        self.stack = QStackedWidget()

        self.manual_screen = ManualScreen(self.hub)
        self.follow_screen = FollowScreen(self.hub)
        self.auto_screen = AutoDriveScreen(self.hub)



        self.stack.addWidget(self.manual_screen)
        self.stack.addWidget(self.follow_screen)
        self.stack.addWidget(self.auto_screen)


        main.addWidget(self.stack)

        self.setLayout(main)

        # ================= BIND =================
        self.btn_manual.clicked.connect(self._show_manual)
        self.btn_follow.clicked.connect(self._show_follow)
        self.btn_auto.clicked.connect(lambda: self.stack.setCurrentWidget(self.auto_screen))

    # ==================================================
    # MODE SWITCH
    # ==================================================

    def _show_manual(self):
        self.hub.set_mode("manual")
        self.stack.setCurrentWidget(self.manual_screen)

    def _show_follow(self):
        self.hub.set_mode("follow")
        self.stack.setCurrentWidget(self.follow_screen)









