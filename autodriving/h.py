import lgpio
import time

SERVO_PIN = 18  # Use GPIO 18

# Open GPIO chip
h = lgpio.gpiochip_open(0)


# Set GPIO pin as output
lgpio.gpio_claim_output(h, SERVO_PIN)

# Function to move servo to specific angle
def set_servo_angle(angle):
	if 0 <= angle <= 180:
		duty_cycle = 2.5 + (angle / 18)
		pulse_width = int((duty_cycle / 100) * 20000)
		#lgpio.tx_servo(h, SERVO_PIN, pulse_width)
		lgpio.tx_pwm(h, SERVO_PIN, 50, duty_cycle)
		print(f"Servo moved to {angle} degrees")
		time.sleep(0.5)
	else:
		print("Angle out of range! Use 0-180 degrees.")

try:
	while True:
		angle = input("Enter angle (0-180) or 'q' to quit: ")
		if angle.lower() == 'q':
			break
		set_servo_angle(int(angle))

except KeyboardInterrupt:
	print("\nExiting...")

finally:
	lgpio.gpiochip_close(h)  # Cleanup GPIO
