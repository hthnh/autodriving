from PyQt5.QtWidgets import QWidget

from PyQt5.QtGui import QPainter, QColor, QPen


import math

class LidarWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.scan_data = []
        self.result = None
        self.setMinimumSize(300, 300)

    def update_data(self, scan_data, result):
        self.scan_data = scan_data
        self.result = result
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        center_x = w // 2
        center_y = h // 2
        radius = min(w, h) // 2 - 20

        # outer circle
        painter.setPen(QPen(QColor(150, 150, 150), 2))
        painter.drawEllipse(center_x - radius, center_y - radius,
                            radius * 2, radius * 2)

        # draw scan points
        painter.setPen(QPen(QColor(0, 255, 0), 3))

        for angle, dist in self.scan_data:
            r = min(dist / 2.0, 1.0) * radius
            rad = math.radians(angle)

            x = center_x + r * math.cos(rad)
            y = center_y - r * math.sin(rad)

            painter.drawPoint(int(x), int(y))

        # draw recommended direction arrow
        if self.result:
            painter.setPen(QPen(QColor(255, 0, 0), 4))
            direction = self.result.get("recommended_direction", "")

            if direction == "left":
                painter.drawLine(center_x, center_y,
                                 center_x - radius // 2, center_y)
            elif direction == "right":
                painter.drawLine(center_x, center_y,
                                 center_x + radius // 2, center_y)
            elif direction == "forward":
                painter.drawLine(center_x, center_y,
                                 center_x, center_y - radius // 2)
