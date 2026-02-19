from PyQt5.QtWidgets import QMessageBox


class ModeMixin:

    def emergency_stop(self):
        self.hub.manual_speed = 0
        self.hub.manual_steer = 90

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

        if level >= 2:
            QMessageBox.information(
                self, "Info",
                f"Level {level} not implemented yet."
            )

        self.apply_ui_rules()

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
