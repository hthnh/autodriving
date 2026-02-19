class StatusMixin:

    def update_status(self):
        txt = f"MODE: {self.mode.upper()}"
        if self.level is not None:
            txt += f" | LEVEL: L{self.level}"

        if self.hub.lane_algo_running:
            txt += " | LANE: RUNNING"

        txt += f" | error={self.hub.lane_state.error:.2f}"

        self.lbl_status.setText(txt)

    def update_follow_debug(self):
        fs = self.hub.follow_state

        self.lbl_follow.setText(
            f"Follow: visible={fs.target_visible} "
            f"error={fs.target_error:.3f} "
            f"steer={fs.final_steer}"
        )
