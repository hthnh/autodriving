class LaneAlgoState:
    def __init__(self):
        self.error = 0.0
        self.confidence = 0.0
        self.timestamp = 0.0

        self.variables = {}
        self.debug_frames = {}
        self.log = []
