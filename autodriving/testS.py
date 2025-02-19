
from picamera2 import Picamera2
import cv2
import numpy as np
import lgpio
import time
from collections import deque

# GPIO Pin Setup
SERVO_PIN = 18  # PWM pin for servo (use hardware PWM)
MOTOR_IN1 = 23  # L298N Motor IN1
MOTOR_IN2 = 24  # L298N Motor IN2
MOTOR_ENA = 25  # L298N PWM Speed Control

# Initialize lgpio chip
chip = lgpio.gpiochip_open(0)


# Set up L298N motor control
lgpio.gpio_claim_output(chip, MOTOR_IN1)
lgpio.gpio_claim_output(chip, MOTOR_IN2)
lgpio.tx_pwm(chip, MOTOR_ENA, 1000, 0)  # PWM for speed control (1kHz, 0% duty)

# Store lane history for smoothing
left_lane_history = deque(maxlen=5)
right_lane_history = deque(maxlen=5)

# Initialize camera
picam2 = Picamera2()
picam2.start()

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
def set_steering(angle):
    if angle < 0:  # Prevent invalid angles
        angle = 0
    elif angle > 30:
        angle = 30

    # Map angle (0 to 30) to duty cycle (2.5% to 12.5%)
    duty_cycle = 2.5 + (angle * (10 / 30))  # Linear mapping

    lgpio.tx_pwm(chip, SERVO_PIN, 50, duty_cycle)  # Set PWM for servo
    print(f"Steering angle: {angle:.2f} -> Servo: {duty_cycle:.2f}% duty cycle")

def process_frame(frame):
    """ Convert frame to grayscale, apply Gaussian blur and Canny edge detection. """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

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
    
    return lines

def separate_lanes(lines, width):
    """ Separate detected lines into left and right lanes based on slope. """
    left_lines = []
    right_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-6)

            if -2.0 < slope < -0.3 and x1 < width // 2 and x2 < width // 2:
                left_lines.append([x1, y1, x2, y2])
            elif 0.3 < slope < 2.0 and x1 > width // 2 and x2 > width // 2:
                right_lines.append([x1, y1, x2, y2])

    return left_lines, right_lines  

def get_lane_midpoint(left_lines, right_lines, height):
    """ Calculate the midpoint of the detected lane, ensuring both lanes exist. """
    if not left_lines or not right_lines:
        return None  

    left_bottom_x = int(np.mean([x1 for x1, _, x2, _ in left_lines] + [x2 for x1, _, x2, _ in left_lines]))
    right_bottom_x = int(np.mean([x1 for x1, _, x2, _ in right_lines] + [x2 for x1, _, x2, _ in right_lines]))

    lane_mid_x = (left_bottom_x + right_bottom_x) // 2
    return lane_mid_x, height - 50  

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
        return previous_angle  

    new_angle = (lane_mid_x - image_mid_x) / image_mid_x * 45
    smoothed_angle = smooth_steering_angle(new_angle)

    return max(min(smoothed_angle, 30), -30)  

while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    processed = process_frame(frame)
    lines = detect_lines(processed)
    left_lines, right_lines = separate_lanes(lines, frame.shape[1])
    lane_mid = get_lane_midpoint(left_lines, right_lines, frame.shape[0])
    angle = calculate_steering_angle(frame, lane_mid[0] if lane_mid else None)

    set_steering(angle)  # Control servo using lgpio
    set_motor(40, "forward")  # Set motor speed to 40% in forward direction

    cv2.imshow('Processed', processed)
    cv2.imshow('Lane Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
lgpio.gpiochip_close(chip)
cv2.destroyAllWindows()
