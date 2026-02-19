#gui/mixins/camera_mixin.py
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt


class CameraMixin:

    def on_target_selected(self, bbox):

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
        scaled_bbox = tuple(int(v) for v in scaled_bbox)

        if self.hub.init_follow_tracker(scaled_bbox):
            print("Tracker initialized")

    def toggle_front_cam(self):
        if self.hub.is_camera_running(1):
            self.hub.stop_camera(1)
            self.btn_front_cam.setText("Front Cam ON")
        else:
            self.hub.start_camera(1)
            self.btn_front_cam.setText("Front Cam OFF")

    def toggle_down_cam(self):
        if self.hub.is_camera_running(0):
            self.hub.stop_camera(0)
            self.btn_down_cam.setText("Down Cam ON")
        else:
            self.hub.start_camera(0)
            self.btn_down_cam.setText("Down Cam OFF")

    def update_live_cameras(self):

        frame_front = self.hub.get_camera_frame(1)
        if frame_front is not None:
            self._show_frame(frame_front, self.lbl_cam_front)

        frame_down = self.hub.get_camera_frame(0)
        if frame_down is not None:
            self._show_frame(frame_down, self.lbl_cam_down)

    def _show_frame(self, frame, label):
        h, w, ch = frame.shape
        bytes_per_line = ch * w

        qimg = QImage(
            frame.data,
            w,
            h,
            bytes_per_line,
            QImage.Format_RGB888
        )

        pix = QPixmap.fromImage(qimg).scaled(
            label.width(),
            label.height(),
            Qt.KeepAspectRatio
        )

        label.setPixmap(pix)
