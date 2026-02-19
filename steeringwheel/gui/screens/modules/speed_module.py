#gui/screens/modules/speed_module.py
from PyQt5.QtWidgets import (
    QGroupBox, QVBoxLayout,
    QPushButton, QLabel
)


class SpeedModule(QGroupBox):

    def __init__(self, hub):
        super().__init__("Speed Control")
        self.hub = hub
        self.enabled = False
        self.target_speed = 0

        self.init_ui()

    def init_ui(self):

        layout = QVBoxLayout()

        self.lbl_speed = QLabel("Target Speed: 0")

        self.btn_plus = QPushButton("Increase Speed")
        self.btn_minus = QPushButton("Decrease Speed")
        self.btn_enable = QPushButton("ENABLE CRUISE")

        layout.addWidget(self.lbl_speed)
        layout.addWidget(self.btn_plus)
        layout.addWidget(self.btn_minus)
        layout.addWidget(self.btn_enable)

        self.setLayout(layout)

        self.btn_plus.clicked.connect(self._inc_speed)
        self.btn_minus.clicked.connect(self._dec_speed)

    def _inc_speed(self):
        self.target_speed += 10
        self.lbl_speed.setText(f"Target Speed: {self.target_speed}")

    def _dec_speed(self):
        self.target_speed -= 10
        self.lbl_speed.setText(f"Target Speed: {self.target_speed}")

    def set_level(self, level):
        self.setEnabled(level >= 1)
