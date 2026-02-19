#gui/mixins/status_mixin.py
class StatusMixin:

    def update_status(self):
        txt = f"MODE: {self.mode.upper()}"
        if self.level is not None:
            txt += f" | LEVEL: L{self.level}"
            
        lane = self.hub.get_lane_status()

        if lane["running"]:
            txt += " | LANE: RUNNING"

        txt += f" | error={lane['error']:.2f}"

        self.lbl_status.setText(txt)

    def update_follow_debug(self):
        fs = self.hub.get_follow_status()

        self.lbl_follow.setText(
            f"Follow: visible={fs['visible']} "
            f"error={fs['error']:.3f} "
            f"steer={fs['steer']}"
        )
