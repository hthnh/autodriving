#target_tracker.py
import cv2

class TargetTracker:
    def __init__(self, state):
        self.state = state
        self.tracker = None

    def init_tracker(self, frame, bbox):

        if frame is None:
            print("No frame")
            return False

        h, w, _ = frame.shape

        # ép kiểu thật sự về int Python
        x = int(bbox[0])
        y = int(bbox[1])
        bw = int(bbox[2])
        bh = int(bbox[3])

        # clamp
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        bw = max(1, min(bw, w - x))
        bh = max(1, min(bh, h - y))

        if bw < 10 or bh < 10:
            print("BBox too small")
            return False

        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        self.tracker = cv2.TrackerKCF_create()

        try:
            # ⚠️ QUAN TRỌNG: truyền tuple int, không float
            ok = self.tracker.init(frame_bgr, (x, y, bw, bh))
            print("Tracker init OK:", x, y, bw, bh)
            return True
        except Exception as e:
            print("Tracker init failed:", e)
            self.tracker = None
            return False



    def update(self, frame):
        if self.tracker is None:
            self.state.target_visible = False
            return
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, box = self.tracker.update(frame_bgr)

        if not ok:
            self.state.target_visible = False
            return

        x, y, w, h = box
        cx = x + w / 2
        frame_center = frame.shape[1] / 2

        error = (cx - frame_center) / frame_center

        self.state.target_error = error
        self.state.target_area = w * h
        self.state.target_visible = True
