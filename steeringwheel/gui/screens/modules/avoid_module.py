#gui/screens/modules/avoid_module.py
from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QLabel


class AvoidModule(QGroupBox):

    def __init__(self, hub):
        super().__init__("Obstacle Avoidance")
        self.hub = hub

        layout = QVBoxLayout()
        layout.addWidget(QLabel("Coming soon..."))

        self.setLayout(layout)

    def set_level(self, level):
        self.setEnabled(level >= 2)
