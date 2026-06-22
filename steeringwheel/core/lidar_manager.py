class LidarManager:
    def __init__(self):
        self.rplidar = RPLidarModule()
        self.tfluna = None   # bật sau

    def get_environment(self):
        scan = self.rplidar.get_scan()
        front = get_front_obstacles(scan)

        return {
            "scan": scan,
            "front_danger": len(front) > 0
        }