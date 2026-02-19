#gui/screens/manual_screen.py
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGridLayout, QGroupBox
)
from PyQt5.QtCore import QTimer


class ManualScreen(QWidget):

    def __init__(self, hub):
        super().__init__()
        self.hub = hub

        self.cur_speed = 0
        self.cur_steer = 90

        self.forward_pressed = False
        self.backward_pressed = False
        self.left_pressed = False
        self.right_pressed = False

        self.init_ui()
        self.init_timers()

    # =================================================
    # UI
    # =================================================

    def init_ui(self):

        main = QVBoxLayout()

        drive_group = QGroupBox("Manual Drive Control")
        grid = QGridLayout()

        self.btn_forward = QPushButton("▲ FORWARD")
        self.btn_backward = QPushButton("▼ BACKWARD")
        self.btn_left = QPushButton("◀ LEFT")
        self.btn_right = QPushButton("RIGHT ▶")
        self.btn_center = QPushButton("● CENTER")

        # Grid layout
        grid.addWidget(self.btn_forward, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_center, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_backward, 2, 1)

        drive_group.setLayout(grid)
        main.addWidget(drive_group)

        # Status
        self.lbl_speed = QLabel("Speed: 0")
        self.lbl_steer = QLabel("Steer: 90")

        main.addWidget(self.lbl_speed)
        main.addWidget(self.lbl_steer)

        # Emergency
        self.btn_emergency = QPushButton("EMERGENCY STOP")
        self.btn_emergency.setStyleSheet(
            "background-color: red; color: white; font-weight: bold; height: 40px;"
        )
        self.btn_emergency.clicked.connect(self._emergency)

        main.addWidget(self.btn_emergency)

        self.setLayout(main)

        # ================= BIND =================

        # Throttle
        self.btn_forward.pressed.connect(lambda: self._set_forward(True))
        self.btn_forward.released.connect(lambda: self._set_forward(False))

        self.btn_backward.pressed.connect(lambda: self._set_backward(True))
        self.btn_backward.released.connect(lambda: self._set_backward(False))

        # Steering hold
        self.btn_left.pressed.connect(lambda: self._set_left(True))
        self.btn_left.released.connect(lambda: self._set_left(False))

        self.btn_right.pressed.connect(lambda: self._set_right(True))
        self.btn_right.released.connect(lambda: self._set_right(False))

        self.btn_center.clicked.connect(self._center_steer)

    # =================================================
    # TIMERS
    # =================================================

    def init_timers(self):

        # Throttle timer
        self.throttle_timer = QTimer()
        self.throttle_timer.timeout.connect(self._update_throttle)
        self.throttle_timer.start(50)

        # Steering timer (100ms step 10)
        self.steer_timer = QTimer()
        self.steer_timer.timeout.connect(self._update_steer)
        self.steer_timer.start(100)

    # =================================================
    # INPUT STATE
    # =================================================

    def _set_forward(self, state):
        self.forward_pressed = state

    def _set_backward(self, state):
        self.backward_pressed = state

    def _set_left(self, state):
        self.left_pressed = state

    def _set_right(self, state):
        self.right_pressed = state

    def _center_steer(self):
        self.cur_steer = 90
        self._push_input()

    # =================================================
    # UPDATE LOGIC
    # =================================================

    def _update_throttle(self):

        if self.forward_pressed:
            self.cur_speed = min(self.cur_speed + 5, 200)

        elif self.backward_pressed:
            self.cur_speed = max(self.cur_speed - 5, -200)

        else:
            if self.cur_speed > 0:
                self.cur_speed -= 10
            elif self.cur_speed < 0:
                self.cur_speed += 10

        self._push_input()

    def _update_steer(self):

        if self.left_pressed:
            self.cur_steer = max(self.cur_steer - 10, 45)

        elif self.right_pressed:
            self.cur_steer = min(self.cur_steer + 10, 135)

        self._push_input()

    def _emergency(self):
        self.cur_speed = 0
        self.cur_steer = 90
        self._push_input()
        self.hub.emergency_stop()

    def _push_input(self):
        self.hub.set_input(self.cur_speed, self.cur_steer)

        self.lbl_speed.setText(f"Speed: {self.cur_speed}")
        self.lbl_steer.setText(f"Steer: {self.cur_steer}")
