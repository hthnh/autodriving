import time
import math


class LidarMixin:

    def update_lidar_view(self):
        scan = self.generate_mock_scan()
        result = self.process_mock_lidar(scan)
        self.lidar.update_data(scan, result)

    def generate_mock_scan(self):
        scan = []
        t = time.time()
        obstacle_center = (math.sin(t) * 90) + 180

        for angle in range(360):
            dist = 2.0
            if abs(angle - obstacle_center) < 10:
                dist = 0.4 + 0.1 * math.sin(t * 3)

            scan.append((angle, dist))

        return scan

    def process_mock_lidar(self, scan):
        front = [d for a, d in scan if 170 < a < 190]
        front_min = min(front) if front else 2.0

        blocked = front_min < self.hub.runtime_params["stop_threshold"]

        direction = "left" if blocked else "forward"

        return {
            "front_min": front_min,
            "front_blocked": blocked,
            "recommended_direction": direction
        }
