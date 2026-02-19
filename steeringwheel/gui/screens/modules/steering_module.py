from PyQt5.QtWidgets import (
    QGroupBox, QVBoxLayout,
    QPushButton, QLabel, QComboBox
)


class SteeringModule(QGroupBox):

    def __init__(self, hub):
        super().__init__("Steering Control")
        self.hub = hub
        self.enabled = False

        self.init_ui()

    def init_ui(self):

        layout = QVBoxLayout()

        self.combo_algo = QComboBox()
        self.combo_algo.addItems(["Lane Keeping (Geometry)"])

        self.btn_enable = QPushButton("ENABLE LANE")
        self.lbl_status = QLabel("Lane: OFF")

        layout.addWidget(self.combo_algo)
        layout.addWidget(self.btn_enable)
        layout.addWidget(self.lbl_status)

        self.setLayout(layout)

        self.btn_enable.clicked.connect(self._toggle_lane)

    def _toggle_lane(self):

        self.enabled = not self.enabled

        if self.enabled:
            self.lbl_status.setText("Lane: ON")
            # sau này:
            # self.hub.enable_lane()
        else:
            self.lbl_status.setText("Lane: OFF")
            # self.hub.disable_lane()

    def set_level(self, level):
        self.setEnabled(level >= 1)
