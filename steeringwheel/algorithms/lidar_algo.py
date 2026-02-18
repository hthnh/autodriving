#lidar_algo.py
class LidarProcessor:
    def __init__(self, front_width=60, side_width=60, stop_threshold=0.5):
        self.front_width = front_width
        self.side_width = side_width
        self.stop_threshold = stop_threshold

    def process(self, scan_data):

        front = []
        left = []
        right = []

        for angle, dist in scan_data:
            # convert angle so 0° = front
            angle = (angle + 180) % 360 - 180

            if -self.front_width/2 <= angle <= self.front_width/2:
                front.append(dist)

            elif self.front_width/2 < angle <= self.front_width/2 + self.side_width:
                left.append(dist)

            elif -self.front_width/2 - self.side_width <= angle < -self.front_width/2:
                right.append(dist)

        front_min = min(front) if front else float("inf")
        left_min = min(left) if left else float("inf")
        right_min = min(right) if right else float("inf")

        front_blocked = front_min < self.stop_threshold

        if front_blocked:
            if left_min > right_min:
                direction = "left"
            elif right_min > left_min:
                direction = "right"
            else:
                direction = "stop"
        else:
            direction = "forward"

        return {
            "front_min": front_min,
            "left_min": left_min,
            "right_min": right_min,
            "front_blocked": front_blocked,
            "recommended_direction": direction
        }
