from control.modes.base_mode import BaseMode


class ManualMode(BaseMode):

    def process(self, input_state):
        return input_state["steer"], input_state["speed"]
