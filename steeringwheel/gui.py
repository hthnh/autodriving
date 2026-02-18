#gui.py
import sys
import cv2
import numpy as np
import math
import time


from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QPen

from PyQt5.QtWidgets import (
    QApplication, QWidget,
    QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton,
    QSlider, QComboBox,
    QGroupBox, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer, QPoint

from control_hub import ControlHub

from algorithms.lane_algo_state import LaneAlgoState


# ================= EXTENDED WINDOW (DUMMY) =================
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


# ================= MAIN GUI =================

class ControlHubGUI(QWidget):
    def __init__(self, hub: ControlHub):
        super().__init__()
        self.hub = hub

        self.mode = "manual"   # manual | auto | follow

        self.level = None

        self.cur_speed = 0
        self.cur_steer = 90

        self.extend_window = None
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_live_cameras)
        self.gui_timer.start(30)   # 30 FPS GUI


        self.init_ui()
        self.init_timer()
        self.apply_ui_rules()

    # ================= UI =================

    def init_ui(self):
        self.setWindowTitle("Autodrive Control Hub")
        self.setFixedSize(1000, 1200)

        main = QVBoxLayout()

        # ===== MODE =====
        mode_box = QHBoxLayout()
        self.btn_manual = QPushButton("MANUAL")
        self.btn_auto = QPushButton("AUTO")
        self.btn_follow = QPushButton("FOLLOW")

        self.btn_manual.clicked.connect(lambda: self.set_mode("manual"))
        self.btn_auto.clicked.connect(lambda: self.set_mode("auto"))
        self.btn_follow.clicked.connect(lambda: self.set_mode("follow"))    

        mode_box.addWidget(self.btn_manual)
        mode_box.addWidget(self.btn_auto)
        mode_box.addWidget(self.btn_follow)
        main.addLayout(mode_box)

        # ===== CAMERA CONTROL =====
        self.cam_group = QGroupBox("Camera Control")
        cam_layout = QHBoxLayout()

        self.btn_front_cam = QPushButton("Front Cam ON")
        self.btn_front_cam.clicked.connect(self.toggle_front_cam)

        self.btn_down_cam = QPushButton("Down Cam ON")
        self.btn_down_cam.clicked.connect(self.toggle_down_cam)

        cam_layout.addWidget(self.btn_front_cam)
        cam_layout.addWidget(self.btn_down_cam)

        self.cam_group.setLayout(cam_layout)
        main.addWidget(self.cam_group)




        # ===== SENSOR VIEW =====
        sensor_layout = QHBoxLayout()

        # Camera View


        self.lbl_cam_front = ClickableCameraLabel()
        self.lbl_cam_front.setText("FRONT CAMERA")
        self.lbl_cam_front.callback = self.on_target_selected
        self.lbl_cam_front.setFixedSize(320, 240)
        self.lbl_cam_front.setStyleSheet("background: black;")





        self.lbl_cam_down = QLabel("Down Camera")
        self.lbl_cam_down.setFixedSize(320, 240)
        self.lbl_cam_down.setStyleSheet("background: black;")

        # Lidar View
        self.lidar_widget = LidarWidget()

        sensor_layout.addWidget(self.lbl_cam_front)
        sensor_layout.addWidget(self.lbl_cam_down)
        sensor_layout.addWidget(self.lidar_widget)

        main.addLayout(sensor_layout)





        # ===== LEVEL =====
        level_group = QGroupBox("Autonomy Level")
        level_layout = QHBoxLayout()

        self.level_buttons = {}
        for i in range(6):
            btn = QPushButton(f"L{i}")
            btn.clicked.connect(lambda _, lv=i: self.set_level(lv))
            self.level_buttons[i] = btn
            level_layout.addWidget(btn)

        level_group.setLayout(level_layout)
        main.addWidget(level_group)

        # ===== SPEED =====
        self.lbl_speed = QLabel("Speed: 0")
        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(-200, 200)
        self.slider_speed.valueChanged.connect(self.on_speed_change)

        main.addWidget(self.lbl_speed)
        main.addWidget(self.slider_speed)

        # ===== STEER =====
        self.lbl_steer = QLabel("Steering: 90")
        self.slider_steer = QSlider(Qt.Horizontal)
        self.slider_steer.setRange(45, 135)
        self.slider_steer.setValue(90)
        self.slider_steer.valueChanged.connect(self.on_steer_change)

        main.addWidget(self.lbl_steer)
        main.addWidget(self.slider_steer)

        # ===== LANE CONTAINER (L1) =====
        self.lane_group = QGroupBox("Lane Keeping Algorithm")
        lane_layout = QVBoxLayout()

        self.combo_algo = QComboBox()
        # map đúng với ControlHub.select_lane_algo
        self.combo_algo.addItems(["geometry"])

        algo_btns = QHBoxLayout()
        self.btn_algo_start = QPushButton("START")
        self.btn_algo_stop = QPushButton("STOP")

        self.btn_algo_start.clicked.connect(self.start_lane_algo)
        self.btn_algo_stop.clicked.connect(self.stop_lane_algo)

        algo_btns.addWidget(self.btn_algo_start)
        algo_btns.addWidget(self.btn_algo_stop)

        self.btn_extend = QPushButton("EXTEND VIEW")
        self.btn_extend.clicked.connect(self.open_extend_view)

        lane_layout.addWidget(QLabel("Algorithm:"))
        lane_layout.addWidget(self.combo_algo)
        lane_layout.addLayout(algo_btns)
        lane_layout.addWidget(self.btn_extend)

        self.lane_group.setLayout(lane_layout)
        main.addWidget(self.lane_group)



        # ===== PARAM PANEL =====
        self.param_group = QGroupBox("Runtime Parameters (Debug)")
        param_layout = QVBoxLayout()

        # Stop threshold
        self.lbl_stop = QLabel("Stop Threshold: 0.50")
        self.slider_stop = QSlider(Qt.Horizontal)
        self.slider_stop.setRange(10, 200)   # 0.10 – 2.00
        self.slider_stop.setValue(50)
        self.slider_stop.valueChanged.connect(self.on_stop_change)

        param_layout.addWidget(self.lbl_stop)
        param_layout.addWidget(self.slider_stop)

        # Steering Kp
        self.lbl_kp = QLabel("Steering Kp: 0.004")
        self.slider_kp = QSlider(Qt.Horizontal)
        self.slider_kp.setRange(1, 50)
        self.slider_kp.setValue(4)
        self.slider_kp.valueChanged.connect(self.on_kp_change)

        param_layout.addWidget(self.lbl_kp)
        param_layout.addWidget(self.slider_kp)

        self.param_group.setLayout(param_layout)
        main.addWidget(self.param_group)




        # ===== STATUS =====
        self.lbl_status = QLabel("")
        main.addWidget(self.lbl_status)
        self.lbl_follow = QLabel("Follow: visible=False error=0.00 steer=90")
        main.addWidget(self.lbl_follow)


        # ===== EMERGENCY STOP =====
        self.btn_emergency = QPushButton("EMERGENCY STOP")
        self.btn_emergency.setStyleSheet(
            "background-color: red; color: white; font-weight: bold; height: 40px;"
        )
        self.btn_emergency.clicked.connect(self.emergency_stop)
        main.addWidget(self.btn_emergency)

        self.setLayout(main)


    def on_target_selected(self, bbox):

        frame = self.hub.camera_manager.get_frame(1)
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

        ok = self.hub.init_follow_tracker(scaled_bbox)

        if ok:
            print("Tracker initialized")





    def toggle_front_cam(self):
        if self.hub.camera_manager.is_running(1):
            self.hub.camera_manager.stop(1)
            self.btn_front_cam.setText("Front Cam ON")
        else:
            self.hub.camera_manager.start(1)
            self.btn_front_cam.setText("Front Cam OFF")

    def toggle_down_cam(self):
        if self.hub.camera_manager.is_running(0):
            self.hub.camera_manager.stop(0)
            self.btn_down_cam.setText("Down Cam ON")
        else:
            self.hub.camera_manager.start(0)
            self.btn_down_cam.setText("Down Cam OFF")

    def update_live_cameras(self):
        # FRONT CAM (1)
        frame_front = self.hub.camera_manager.get_frame(1)
        if frame_front is not None:
            self._show_frame(frame_front, self.lbl_cam_front)

        # DOWN CAM (0)
        frame_down = self.hub.camera_manager.get_frame(0)
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


    # ================= MODE / LEVEL =================

    def emergency_stop(self):
        self.cur_speed = 0
        self.cur_steer = 90
        self.slider_speed.setValue(0)
        self.slider_steer.setValue(90)
        self.hub.send_manual(90, 0)

    def set_mode(self, mode):
        self.mode = mode
        self.level = None
        if mode == "follow":
            self.hub.start_follow()
        else:
            self.hub.stop_follow()

        self.apply_ui_rules()

    def set_level(self, level):
        if self.mode != "auto":
            return

        self.level = level
        self.hub.set_level(level)
        self.apply_ui_rules()

        if level >= 2:
            QMessageBox.information(
                self, "Info",
                f"Level {level} not implemented yet."
            )

    # ================= UI RULE ENGINE =================

    def apply_ui_rules(self):
        is_manual = self.mode == "manual"
        is_auto = self.mode == "auto"
        is_follow = self.mode == "follow"


        for btn in self.level_buttons.values():
            btn.setEnabled(is_auto)

        self.slider_speed.setEnabled(False)
        self.slider_steer.setEnabled(False)
        self.lane_group.setVisible(False)

        if is_manual:
            self.slider_speed.setEnabled(True)
            self.slider_steer.setEnabled(True)
            self.hub.set_level(0)
            self.hub.active_source = "manual"

        if is_auto and self.level is not None:
            self.hub.active_source = "autodrive"

            if self.level == 0:
                self.slider_speed.setEnabled(True)
                self.slider_steer.setEnabled(True)

            elif self.level == 1:
                self.slider_speed.setEnabled(True)
                self.slider_steer.setEnabled(False)
                self.lane_group.setVisible(True)
        if is_follow:
            self.slider_speed.setEnabled(True)
            self.slider_steer.setEnabled(False)
            self.hub.active_source = "autodrive"

        self.update_status()

    # ================= INPUT =================

    def on_speed_change(self, v):
        self.cur_speed = v
        self.lbl_speed.setText(f"Speed: {v}")

    def on_steer_change(self, v):
        self.cur_steer = v
        self.lbl_steer.setText(f"Steering: {v}")

    # ================= LANE ALGO CONTROL =================

    def start_lane_algo(self):
        algo = self.combo_algo.currentText()
        ok = self.hub.select_lane_algo(algo)
        if not ok:
            QMessageBox.warning(self, "Warning", "Lane algorithm already running")
            return
        self.hub.start_lane_algo()

    def stop_lane_algo(self):
        self.hub.stop_lane_algo()

    # ================= EXTEND =================

    def open_extend_view(self):
        if self.extend_window is None:
            self.extend_window = LaneExtendWindow(self.hub)
        self.extend_window.show()

    # ================= CONTROL LOOP =================

    def init_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.control_tick)
        self.timer.start(50)

    def control_tick(self):
        if self.mode == "manual":
            self.hub.send_manual(self.cur_steer, self.cur_speed)

        elif self.mode == "auto":
            if self.level == 0:
                self.hub.send_manual(self.cur_steer, self.cur_speed)

            elif self.level == 1:
                # speed từ manual, steer do lane algo
                self.hub.send_manual(self.cur_steer, self.cur_speed)

        elif self.mode == "follow":
            # tạm thời giữ speed manual
            self.hub.send_manual(self.cur_steer, self.cur_speed)
        

        self.update_lidar_view()
        self.update_follow_debug()

    def update_follow_debug(self):
        fs = self.hub.follow_state

        self.lbl_follow.setText(
            f"Follow: visible={fs.target_visible} "
            f"error={fs.target_error:.3f} "
            f"steer={fs.final_steer}"
        )



    # ================= STATUS =================

    def update_status(self):
        txt = f"MODE: {self.mode.upper()}"
        if self.level is not None:
            txt += f" | LEVEL: L{self.level}"
        if self.hub.lane_algo_running:
            txt += " | LANE: RUNNING"
        txt += f" | error={self.hub.lane_state.error:.2f}"
        
        self.lbl_status.setText(txt)

    def on_stop_change(self, v):
        val = v / 100.0
        self.lbl_stop.setText(f"Stop Threshold: {val:.2f}")
        self.hub.runtime_params["stop_threshold"] = val

    def on_kp_change(self, v):
        val = v / 1000.0
        self.lbl_kp.setText(f"Steering Kp: {val:.3f}")
        self.hub.runtime_params["kp_steering"] = val




    def update_lidar_view(self):
        # mock lidar for now
        scan = self.generate_mock_scan()
        result = self.process_mock_lidar(scan)
        self.lidar_widget.update_data(scan, result)


    def generate_mock_scan(self):
        scan = []
        t = time.time()

        # obstacle quay vòng
        obstacle_center = (math.sin(t) * 90) + 180   # dao động quanh phía trước

        for angle in range(360):
            dist = 2.0

            # tạo obstacle rộng 20°
            if abs(angle - obstacle_center) < 10:
                dist = 0.4 + 0.1 * math.sin(t * 3)

            scan.append((angle, dist))

        return scan


    def process_mock_lidar(self, scan):
        front = [d for a, d in scan if 170 < a < 190]
        front_min = min(front) if front else 2.0

        blocked = front_min < self.hub.runtime_params["stop_threshold"]

        if blocked:
            direction = "left"
        else:
            direction = "forward"

        return {
            "front_min": front_min,
            "front_blocked": blocked,
            "recommended_direction": direction
        }





# ================= MAIN =================

def main():
    hub = ControlHub()

    app = QApplication(sys.argv)
    gui = ControlHubGUI(hub)
    gui.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
