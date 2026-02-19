from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout, QHBoxLayout,
    QLabel
)

class LaneExtendWindow(QWidget):
    def __init__(self, hub):
        super().__init__()
        self.hub = hub

        self.setWindowTitle("Lane Algorithm – Extended View")
        self.setFixedSize(700, 520)

        layout = QVBoxLayout()

        # ===== STATUS =====
        self.lbl_error = QLabel("Error: 0.000")
        self.lbl_conf = QLabel("Confidence: 0.00")

        layout.addWidget(self.lbl_error)
        layout.addWidget(self.lbl_conf)

        # ===== IMAGE VIEWS =====
        img_layout = QHBoxLayout()

        self.lbl_raw = QLabel("RAW")
        self.lbl_raw.setFixedSize(320, 240)
        self.lbl_raw.setStyleSheet("background: black;")

        self.lbl_edges = QLabel("EDGES")
        self.lbl_edges.setFixedSize(320, 240)
        self.lbl_edges.setStyleSheet("background: black;")

        img_layout.addWidget(self.lbl_raw)
        img_layout.addWidget(self.lbl_edges)

        layout.addLayout(img_layout)

        self.setLayout(layout)

        # ===== TIMER =====
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_view)
        self.timer.start(30)   # 10 FPS UI

    # ================= UPDATE =================

    def refresh_view(self):
        state = self.hub.lane_state

        # --- text ---
        self.lbl_error.setText(f"Error: {state.error:.3f}")
        self.lbl_conf.setText(f"Confidence: {state.confidence:.2f}")

        if not hasattr(state, "debug_frames"):
            return

        frames = state.debug_frames

        # --- RAW IMAGE ---
        if "raw" in frames:
            self._show_rgb(frames["raw"], self.lbl_raw)

        # --- EDGES IMAGE ---
        if "edges" in frames:
            self._show_gray(frames["edges"], self.lbl_edges)

    # ================= HELPERS =================

    def _show_rgb(self, img, label):
        # img: RGB888 numpy array
        h, w, ch = img.shape
        bytes_per_line = ch * w
        qimg = QImage(
            img.data, w, h,
            bytes_per_line,
            QImage.Format_RGB888
        )
        pix = QPixmap.fromImage(qimg).scaled(
            label.width(),
            label.height(),
            Qt.KeepAspectRatio
        )
        label.setPixmap(pix)

    def _show_gray(self, img, label):
        # img: grayscale numpy array
        h, w = img.shape
        qimg = QImage(
            img.data, w, h,
            w,
            QImage.Format_Grayscale8
        )
        pix = QPixmap.fromImage(qimg).scaled(
            label.width(),
            label.height(),
            Qt.KeepAspectRatio
        )
        label.setPixmap(pix)
