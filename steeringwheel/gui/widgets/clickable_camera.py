
from PyQt5.QtWidgets import QLabel

from PyQt5.QtGui import QPainter, QColor, QPen




class ClickableCameraLabel(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.start_point = None
        self.end_point = None
        self.drawing = False
        self.callback = None

    def mousePressEvent(self, event):
        self.start_point = event.pos()
        self.end_point = event.pos()
        self.drawing = True

    def mouseMoveEvent(self, event):
        if self.drawing:
            self.end_point = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        self.drawing = False
        self.end_point = event.pos()
        self.update()

        if self.callback:
            x1 = self.start_point.x()
            y1 = self.start_point.y()
            x2 = self.end_point.x()
            y2 = self.end_point.y()

            x = min(x1, x2)
            y = min(y1, y2)
            w = abs(x1 - x2)
            h = abs(y1 - y2)

            self.callback((x, y, w, h))

    def paintEvent(self, event):
        super().paintEvent(event)

        if self.drawing and self.start_point and self.end_point:
            painter = QPainter(self)
            painter.setPen(QPen(QColor(255, 0, 0), 2))
            rect_x = min(self.start_point.x(), self.end_point.x())
            rect_y = min(self.start_point.y(), self.end_point.y())
            rect_w = abs(self.start_point.x() - self.end_point.x())
            rect_h = abs(self.start_point.y() - self.end_point.y())
            painter.drawRect(rect_x, rect_y, rect_w, rect_h)
