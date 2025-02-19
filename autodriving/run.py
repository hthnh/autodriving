
from picamera2 import Picamera2
import cv2
import numpy as np
import lgpio
import time
from collections import deque
import lgpio

# Open GPIO chip
chip = lgpio.gpiochip_open(0)

# Enable hardware PWM on GPIO 18 (50Hz)
lgpio.tx_pwm(chip, SERVO_PIN, 50, 7.5)  # Start at center (7.5% duty cycle)



# GPIO Pin Setup
SERVO_PIN = 18  # PWM pin for servo (use hardware PWM)
MOTOR_IN1 = 23  # L298N Motor IN1
MOTOR_IN2 = 24  # L298N Motor IN2
MOTOR_ENA = 25  # L298N PWM Speed Control

# Initialize lgpio chip
chip = lgpio.gpiochip_open(0)

# Set GPIO pin as output
lgpio.gpio_claim_output(chip, SERVO_PIN)


# Set up L298N motor control
lgpio.gpio_claim_output(chip, MOTOR_IN1)
lgpio.gpio_claim_output(chip, MOTOR_IN2)
lgpio.gpio_claim_output(chip, MOTOR_ENA)

# Store lane history for smoothing
left_lane_history = deque(maxlen=5)
right_lane_history = deque(maxlen=5)

# Initialize camera
picam2 = Picamera2()
picam2.start()

def process_frame(frame):
    """ Convert frame to grayscale, apply Gaussian blur and Canny edge detection. """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)  # Adjusted thresholds for better lane detection

    # Apply region mask to focus only on lower half
    height = edges.shape[0]
    mask = np.zeros_like(edges)
    mask[int(height * 0.6):, :] = 255  
    cropped_edges = cv2.bitwise_and(edges, mask)

    return cropped_edges

def detect_lines(cropped_edges):
    """ Detect lines using Hough Transform. """
    lines = cv2.HoughLinesP(
        cropped_edges, 
        1, 
        np.pi/180, 
        threshold=20,  
        minLineLength=30, 
        maxLineGap=100  
    )
    
    if lines is None:
        print("?? No lane lines detected!")
    else:
        print(f"? Detected {len(lines)} lines")

    return lines


def separate_lanes(lines, width):
    """ Separate detected lines into left and right lanes based on slope. """
    left_lines = []
    right_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Avoid division by zero

            # Filter lines based on slope and position
            if -2.0 < slope < -0.3 and x1 < width // 2 and x2 < width // 2:
                left_lines.append([x1, y1, x2, y2])  # Store as list
            elif 0.3 < slope < 2.0 and x1 > width // 2 and x2 > width // 2:
                right_lines.append([x1, y1, x2, y2])  # Store as list

    # Append new lanes to history if detected
    if left_lines:
        left_lane_history.append(np.mean(left_lines, axis=0).astype(int).tolist())  # Convert to list
    if right_lines:
        right_lane_history.append(np.mean(right_lines, axis=0).astype(int).tolist())  # Convert to list

    # Compute average lane over history and ensure it's still a list
    avg_left = np.mean(list(left_lane_history), axis=0).astype(int).tolist() if left_lane_history else []
    avg_right = np.mean(list(right_lane_history), axis=0).astype(int).tolist() if right_lane_history else []

    return [avg_left], [avg_right]  # Ensure returned values are lists of points

def get_lane_midpoint(left_lines, right_lines, height):
    """ Calculate the midpoint of the detected lane, ensuring both lanes exist. """
    
    if not left_lines or not right_lines:  # Ensure both lanes exist
        return None  

    # Extract x-coordinates safely
    left_x_values = []
    right_x_values = []

    for line in left_lines:
        if isinstance(line, (list, np.ndarray)) and len(line) == 4:  # Ensure valid format
            x1, y1, x2, y2 = line
            left_x_values.extend([x1, x2])

    for line in right_lines:
        if isinstance(line, (list, np.ndarray)) and len(line) == 4:  # Ensure valid format
            x1, y1, x2, y2 = line
            right_x_values.extend([x1, x2])

    if not left_x_values or not right_x_values:  # Ensure valid values exist
        return None  

    left_bottom_x = int(np.mean(left_x_values))
    right_bottom_x = int(np.mean(right_x_values))

    lane_mid_x = (left_bottom_x + right_bottom_x) // 2

    return lane_mid_x, height - 50  # Position midpoint slightly above bottom


 


previous_angle = 0

def smooth_steering_angle(new_angle, alpha=0.2):
    """ Apply smoothing to reduce sudden changes in steering angle. """
    global previous_angle
    smoothed_angle = alpha * new_angle + (1 - alpha) * previous_angle
    previous_angle = smoothed_angle
    return smoothed_angle

def calculate_steering_angle(frame, lane_mid_x):
    """ Compute steering angle based on lane midpoint deviation from image center. """
    height, width, _ = frame.shape
    image_mid_x = width // 2

    if lane_mid_x is None:
        return previous_angle  # Maintain previous angle if no lanes detected

    new_angle = (lane_mid_x - image_mid_x) / image_mid_x * 45
    smoothed_angle = smooth_steering_angle(new_angle)

    return max(min(smoothed_angle, 30), -30)  
    
    
    
def set_motor(speed, direction="forward"):
    """Control L298N motor driver using lgpio."""
    if direction == "forward":
        lgpio.gpio_write(chip, MOTOR_IN1, 1)
        lgpio.gpio_write(chip, MOTOR_IN2, 0)
    elif direction == "reverse":
        lgpio.gpio_write(chip, MOTOR_IN1, 0)
        lgpio.gpio_write(chip, MOTOR_IN2, 1)
    else:
        lgpio.gpio_write(chip, MOTOR_IN1, 0)
        lgpio.gpio_write(chip, MOTOR_IN2, 0)

    lgpio.tx_pwm(chip, MOTOR_ENA, 1000, speed)  # Adjust speed (0-100%)
last_servo_angle = None  # Store last sent servo position


last_servo_angle = None  # Store last angle to avoid redundant updates
last_update_time = time.time()  # Track last update time


def set_steering(angle):
    """Use Raspberry Pi Hardware PWM to prevent jitter under high CPU load."""
    global last_servo_angle, last_update_time

    angle = smooth_steering_angle(angle)
    mapped_angle = (angle + 30) * (40 / 60)

    # Convert to microseconds (500탎 - 2500탎 range)
    pulse_width_us = int(500 + (mapped_angle * (2000 / 40)))  

    current_time = time.time()
    if (last_servo_angle is None or abs(mapped_angle - last_servo_angle) > 2) and (current_time - last_update_time > 1):
        lgpio.tx_servo(chip, SERVO_PIN, pulse_width_us)  # ? Use Hardware PWM instead of software
        last_servo_angle = mapped_angle
        last_update_time = current_time
        print(f"? Steering updated: {angle:.2f}� -> Servo Pulse: {pulse_width_us}탎")
    else:
        print(f"? Skipping redundant PWM update: {pulse_width_us}탎, cooldown active")

while True:
    # Capture and process frame
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    processed = process_frame(frame)
    lines = detect_lines(processed)
    left_lines, right_lines = separate_lanes(lines, frame.shape[1])
    lane_mid = get_lane_midpoint(left_lines, right_lines, frame.shape[0])
    angle = calculate_steering_angle(frame, lane_mid[0] if lane_mid else None)

    set_steering(angle)  # Control servo using lgpio
    set_motor(10, "forward")  # Set motor speed to 40% in forward direction

    # Draw detected lane lines
    if left_lines and len(left_lines) == 4:
        cv2.line(frame, (left_lines[0], left_lines[1]), (left_lines[2], left_lines[3]), (255, 0, 0), 2)

    if right_lines and len(right_lines) == 4:
        cv2.line(frame, (right_lines[0], right_lines[1]), (right_lines[2], right_lines[3]), (0, 255, 0), 2)

    # Draw lane midpoint and reference
    if lane_mid:
        cv2.circle(frame, lane_mid, 5, (0, 0, 255), -1)  
        cv2.circle(frame, (frame.shape[1] // 2, frame.shape[0] - 50), 5, (255, 255, 0), -1)  
        cv2.line(frame, lane_mid, (frame.shape[1] // 2, frame.shape[0] - 50), (255, 255, 255), 2)  

    # Display steering angle
    cv2.putText(frame, f"Steering Angle: {angle:.2f}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    print(f"Steering Angle: {angle:.2f}")

    # Show frames
    cv2.imshow('Processed', processed)
    cv2.imshow('Lane Detection', frame)

    # Exit loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
