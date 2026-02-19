from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox
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

        self.init_ui()
        self.init_timer()

    # ==========================================
    # UI
    # ==========================================

    def init_ui(self):

        main = QVBoxLayout()

        drive_group = QGroupBox("Manual Drive Control")
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
        self.btn_emergency.clicked.connect(self._emergency)

        drive_layout.addWidget(self.btn_emergency)

        drive_group.setLayout(drive_layout)
        main.addWidget(drive_group)

        self.setLayout(main)

        # Bind buttons
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

    # ==========================================
    # Logic
    # ==========================================

    def init_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_input)
        self.timer.start(50)

    def _set_forward(self, state):
        self.forward_pressed = state

    def _set_backward(self, state):
        self.backward_pressed = state

    def _set_steer(self, value):
        self.cur_steer = value
        self._push_input()

    def _emergency(self):
        self.cur_speed = 0
        self.cur_steer = 90
        self._push_input()
        self.hub.emergency_stop()

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

        self._push_input()

    def _push_input(self):
        self.hub.set_input(self.cur_speed, self.cur_steer)

        self.lbl_speed.setText(f"Speed: {self.cur_speed}")
        self.lbl_steer.setText(f"Steer: {self.cur_steer}")
