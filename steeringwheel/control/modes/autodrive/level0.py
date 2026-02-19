from control.modes.base_mode import BaseMode


class Level0Mode(BaseMode):

    def process(self, input_state):
        return input_state["steer"], input_state["speed"]
