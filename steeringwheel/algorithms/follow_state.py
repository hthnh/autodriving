class FollowState:
    def __init__(self):
        self.target_error = 0.0
        self.target_visible = False
        self.target_area = 0.0

        self.front_blocked = False
        self.recommended_direction = "forward"

        self.final_steer = 90
        self.final_speed = 0

        self.debug = {}
