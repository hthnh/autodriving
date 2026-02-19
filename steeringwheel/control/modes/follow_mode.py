from control.modes.base_mode import BaseMode
from core.behavior import follow_arbitration


class FollowMode(BaseMode):

    def process(self, input_state):

        self.hub.lidar_processor.stop_threshold = \
            self.hub.runtime_params["stop_threshold"]

        frame = self.hub.camera_manager.get_frame(1)
        self.hub.target_tracker.update(frame)

        scan = [(a, 2.0) for a in range(360)]
        lidar_result = self.hub.lidar_processor.process(scan)

        steer, speed = follow_arbitration(
            self.hub.follow_state,
            lidar_result,
            self.hub.runtime_params
        )

        return steer, speed
    