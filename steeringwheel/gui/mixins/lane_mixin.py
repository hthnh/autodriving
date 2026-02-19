class LaneMixin:

    def start_lane_algo(self):
        algo = self.combo_algo.currentText()
        ok = self.hub.select_lane_algo(algo)
        if ok:
            self.hub.start_lane_algo()

    def stop_lane_algo(self):
        self.hub.stop_lane_algo()

    def open_extend_view(self):
        if self.lane_extend is None:
            from gui.widgets.lane_extend_window import LaneExtendWindow
            self.extend_window = LaneExtendWindow(self.hub)

        self.extend_window.show()
