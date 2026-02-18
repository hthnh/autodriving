def follow_arbitration(follow_state, lidar_result, params):

    if not follow_state.target_visible:
        follow_state.final_steer = 90
        follow_state.final_speed = 0
        return 90, 0

    kp = params.get("kp_steering", 0.004)

    # error ∈ [-1, 1]
    error = follow_state.target_error

    steer = 90 + error * 45   # tối đa lệch 45 độ
    speed = 60                # tạm thời fix

    follow_state.final_steer = int(steer)
    follow_state.final_speed = int(speed)

    return steer, speed
