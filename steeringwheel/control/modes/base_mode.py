#control/modes/base_mode.py
class BaseMode:
    def __init__(self, hub):
        self.hub = hub

    def process(self, input_state):
        """
        Must return (steer, speed)
        """
        raise NotImplementedError
