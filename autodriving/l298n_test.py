import lgpio
import time

# Define GPIO pins for L298N
ENA = 25  # PWM Speed Control
IN1 = 23  # Motor Direction
IN2 = 24  # Motor Direction

# Open GPIO chip
h = lgpio.gpiochip_open(0)

# Set up GPIO pins as outputs
lgpio.gpio_claim_output(h, IN1)
lgpio.gpio_claim_output(h, IN2)
lgpio.gpio_claim_output(h, ENA)

def set_motor(speed, direction="forward"):
    """
    Controls the L298N Motor Driver.
    speed: 0 to 100 (percentage)
    direction: "forward" or "backward"
    """
    pwm_duty_cycle = max(0, min(speed, 100))  # Ensure duty cycle is within 0-100

    if direction == "forward":
        lgpio.gpio_write(h, IN1, 1)  # IN1 HIGH
        lgpio.gpio_write(h, IN2, 0)  # IN2 LOW
    elif direction == "backward":
        lgpio.gpio_write(h, IN1, 0)  # IN1 LOW
        lgpio.gpio_write(h, IN2, 1)  # IN2 HIGH
    else:
        lgpio.gpio_write(h, IN1, 0)
        lgpio.gpio_write(h, IN2, 0)

    # Set PWM speed with correct duty cycle range (0-100)
    lgpio.tx_pwm(h, ENA, 100, pwm_duty_cycle)  # 100Hz frequency

try:
    while True:
        cmd = input("Enter command (f = forward, b = backward, s = stop, q = quit): ")
        
        if cmd == "f":
            set_motor(20, "forward")  # 80% speed forward
            print("Motor moving forward")
        elif cmd == "b":
            set_motor(10, "backward")  # 80% speed backward
            print("Motor moving backward")
        elif cmd == "s":
            set_motor(0)  # Stop motor
            print("Motor stopped")
        elif cmd == "q":
            break

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    set_motor(0)  # Stop the motor
    lgpio.gpiochip_close(h)  # Cleanup GPIO
