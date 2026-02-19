#control/modes/autodrive/level1.py
from control.modes.base_mode import BaseMode


class Level1Mode(BaseMode):

    def process(self, input_state):

        # speed từ user
        speed = input_state["speed"]

        # lane steering
        if self.hub.lane_algo_running:
            self.hub.lane_algo.update()

            error = self.hub.lane_state.error
            kp = self.hub.runtime_params["kp_steering"]

            steer = 90 + error * kp * 100
        else:
            steer = input_state["steer"]

        return steer, speed
