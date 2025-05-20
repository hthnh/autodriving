# control_processor.py
# Reads raw NRF data from Redis Stream, processes it into vehicle commands,
# and publishes these processed commands to another Redis Stream.

import redis
import time
import sys
import math

# --- Redis Configuration ---
REDIS_HOST = 'localhost'
REDIS_PORT = 6379
redis_client = None

RAW_NRF_STREAM_NAME = 'nrf:raw_data'               # Stream to read raw NRF data from
PROCESSED_COMMAND_STREAM_NAME = 'vehicle:commands_processed' # Stream to publish processed commands to

CONSUMER_GROUP_NAME = 'control_processor_group'    # Consumer group for reading raw NRF data
CONSUMER_NAME = 'processor_instance_1'           # Name of this consumer instance

# --- Control Constants (same as in the previous NRF processing logic) ---
# Motor
STOP = 0
FORWARD = 1
BACKWARD = 2
# Speed Levels (Index 0=Stop, 1=Lowest, 5=Highest)
SPEED_LEVELS = [0, 60, 100, 150, 200, 255] # Tune these speeds as needed
# Joystick
JOY_MIN = 0
JOY_MAX = 10
JOY_CENTER = 5
# Servo
SERVO_MIN_ANGLE0 = 0
SERVO_MAX_ANGLE0 = 180

SERVO_MIN_ANGLE1 = 35
SERVO_MAX_ANGLE1 = 135
SERVO_CENTER_ANGLE = 90
# Buttons (Indices based on how nrf_receiver.py structures the data)
# Assuming nrf_receiver.py sends btn0, btn1, ...
EMERGENCY_STOP_BTN_FIELD = 'btn0' # Button 1 is at index 0
RESET_GIMBAL_BTN_FIELD = 'btn1' # Button 2 is at index 1

# --- Helper Function ---
def map_range(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another."""
    if in_min == in_max: return out_min
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def setup_redis_client():
    """Initializes the connection to the Redis server."""
    global redis_client
    try:
        print(f"ControlProcessor: Connecting to Redis at {REDIS_HOST}:{REDIS_PORT}...")
        redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=False) # False for raw bytes
        redis_client.ping()
        print("ControlProcessor: Redis connection established.")
        
        # Create consumer group if it doesn't exist for the raw NRF stream
        # This should ideally be done once, but try-except handles it
        try:
            redis_client.xgroup_create(
                name=RAW_NRF_STREAM_NAME,
                groupname=CONSUMER_GROUP_NAME,
                id='0', # Start from the beginning of the stream
                mkstream=True # Create the stream if it doesn't exist
            )
            print(f"ControlProcessor: Consumer group '{CONSUMER_GROUP_NAME}' created for stream '{RAW_NRF_STREAM_NAME}'.")
        except redis.exceptions.ResponseError as e:
            if "BUSYGROUP" in str(e):
                print(f"ControlProcessor: Consumer group '{CONSUMER_GROUP_NAME}' already exists for stream '{RAW_NRF_STREAM_NAME}'.")
            else:
                raise # Re-raise other Redis errors
        return True
    except Exception as e:
        print(f"ControlProcessor: Failed to connect to or setup Redis: {e}")
        redis_client = None
        return False

def process_raw_nrf_data(raw_data_dict):
    """
    Processes raw NRF data (from Redis) and returns a dictionary of processed vehicle commands.
    raw_data_dict is expected to have fields like b'btn0', b'joyX1', etc. with byte string values.
    """
    try:
        # Convert byte string values from Redis to appropriate types
        buttons_pressed = [int(raw_data_dict.get(f'btn{i}'.encode(), b'0')) for i in range(11)]
        joyX1 = int(raw_data_dict.get(b'joyX1', b'5'))
        joyY1 = int(raw_data_dict.get(b'joyY1', b'5'))
        joyX2 = int(raw_data_dict.get(b'joyX2', b'5'))
        joyY2 = int(raw_data_dict.get(b'joyY2', b'5'))
        # nrf_timestamp = float(raw_data_dict.get(b'timestamp', b'0')) # Original timestamp from nrf_receiver

    except ValueError as e:
        print(f"ControlProcessor: Error converting raw NRF data: {e}. Data: {raw_data_dict}")
        return None # Indicate processing error

    e_stop_active = bool(buttons_pressed[0]) # Index 0 for EMERGENCY_STOP_BTN_FIELD 'btn0'
    reset_gimbal_active = bool(buttons_pressed[1]) # Index 1 for RESET_GIMBAL_BTN_FIELD 'btn1'

    # Initialize processed commands
    motor_left_dir = STOP
    motor_left_speed = 0
    motor_right_dir = STOP
    motor_right_speed = 0
    servo1_angle = SERVO_CENTER_ANGLE # Default
    servo2_angle = SERVO_CENTER_ANGLE # Default

    # --- Motor Control ---
    if e_stop_active:
        motor_left_dir = STOP
        motor_left_speed = 0
        motor_right_dir = STOP
        motor_right_speed = 0
        print("ControlProcessor: E-STOP ACTIVE!")
    else:
        norm_y = map_range(joyY1, JOY_MIN, JOY_MAX, 1.0, -1.0)
        norm_x = map_range(joyX1, JOY_MIN, JOY_MAX, -1.0, 1.0)

        raw_left = norm_y - norm_x
        raw_right = norm_y + norm_x

        clamped_left = max(-1.0, min(1.0, raw_left))
        clamped_right = max(-1.0, min(1.0, raw_right))

        motor_left_dir = FORWARD if clamped_left >= 0 else BACKWARD
        motor_right_dir = FORWARD if clamped_right >= 0 else BACKWARD
        
        # Ensure len(SPEED_LEVELS) - 1 is the max index for scaling
        max_level_idx = len(SPEED_LEVELS) - 1
        left_level_idx = math.ceil(abs(clamped_left) * max_level_idx)
        right_level_idx = math.ceil(abs(clamped_right) * max_level_idx)
        
        # Ensure indices are within bounds
        left_level_idx = min(max(0, left_level_idx), max_level_idx)
        right_level_idx = min(max(0, right_level_idx), max_level_idx)

        motor_left_speed = SPEED_LEVELS[left_level_idx]
        motor_right_speed = SPEED_LEVELS[right_level_idx]

        if left_level_idx == 0: motor_left_dir = STOP
        if right_level_idx == 0: motor_right_dir = STOP
        
    # --- Servo Control ---
    if reset_gimbal_active:
        servo1_angle = SERVO_CENTER_ANGLE
        servo2_angle = SERVO_CENTER_ANGLE
        print("ControlProcessor: GIMBAL RESET ACTIVE!")
    else:
        # Map Joystick 2 to servo angles
        servo2_angle = int(map_range(joyX2, JOY_MIN, JOY_MAX, SERVO_MIN_ANGLE1, SERVO_MAX_ANGLE1))
        servo1_angle = int(map_range(joyY2, JOY_MAX, JOY_MIN, SERVO_MIN_ANGLE0, SERVO_MAX_ANGLE0)) # Assuming Y controls servo 2

        # Clamp angles to ensure they are within valid servo range
        servo2_angle = max(SERVO_MIN_ANGLE1, min(SERVO_MAX_ANGLE1, servo2_angle))
        servo1_angle = max(SERVO_MIN_ANGLE0, min(SERVO_MAX_ANGLE0, servo1_angle))

    processed_commands = {
        'motor_left_dir': motor_left_dir,
        'motor_left_speed': motor_left_speed,
        'motor_right_dir': motor_right_dir,
        'motor_right_speed': motor_right_speed,
        'servo1_angle': servo1_angle,
        'servo2_angle': servo2_angle,
        'e_stop_active': int(e_stop_active), # Send as int
        'timestamp': time.time() # Timestamp of when processing is done
    }
    return processed_commands

def publish_processed_commands(commands_dict):
    """Publishes processed commands to the designated Redis Stream."""
    if redis_client and commands_dict:
        try:
            # Convert all values to strings for xadd if they are not already,
            # or ensure redis-py handles type conversion for dict values.
            # redis-py's xadd with a dict should handle int/float fine.
            message_id = redis_client.xadd(PROCESSED_COMMAND_STREAM_NAME, commands_dict)
            
            # print(f"ControlProcessor: Published processed cmd (ID: {message_id}): {commands_dict}") # Debug
            return message_id
        except redis.exceptions.RedisError as e:
            print(f"ControlProcessor: Redis error publishing processed commands: {e}")
        except Exception as e:
            print(f"ControlProcessor: Error publishing processed commands: {e}")
    return None


def main_loop():
    """Continuously reads raw NRF data, processes it, and publishes processed commands."""
    if not redis_client:
        print("ControlProcessor: Redis client not initialized. Exiting.")
        return

    print(f"ControlProcessor: Listening to Redis Stream '{RAW_NRF_STREAM_NAME}' using group '{CONSUMER_GROUP_NAME}'...")
    
    # '>' ID means only new messages arriving after this consumer connected and joined the group.
    # Or, if the consumer was offline and reconnects, it gets messages it missed (if not acked by others).
    # For a single consumer script like this, it effectively processes messages it hasn't seen.
    stream_to_read = {RAW_NRF_STREAM_NAME: '>'} 

    while True:
        try:
            # Read messages from the stream using the consumer group
            # BLOCK for 1 second if no messages, count=1 to process one message at a time
            # (adjust count for batching if needed)
            messages = redis_client.xreadgroup(
                groupname=CONSUMER_GROUP_NAME,
                consumername=CONSUMER_NAME,
                streams=stream_to_read,
                count=1,
                block=1000 # milliseconds
            )

            if messages:
                for stream_name, message_list in messages:
                    for message_id, message_data in message_list:
                        # message_data is a dict with keys as bytes and values as bytes
                        # print(f"ControlProcessor: Received raw NRF (ID: {message_id.decode()}): {message_data}") # Debug
                        
                        processed_cmds = process_raw_nrf_data(message_data)
                        
                        if processed_cmds:
                            publish_processed_commands(processed_cmds)
                        
                        # Acknowledge the message so it's not re-delivered (important for consumer groups)
                        # redis_client.xack(RAW_NRF_STREAM_NAME, CONSUMER_GROUP_NAME, message_id)
                        # For now, let's assume processing is quick and we don't need complex ack/pending logic.
                        # If processing could fail and needs retry, XACK is crucial.
                        # If this is the ONLY consumer and we just want to read everything,
                        # we could use XREAD without a group and manage the last seen ID ourselves.
                        # But consumer group is more robust if we ever want to scale or handle crashes.
                        # For simplicity here, if XREADGROUP is used, XACK is good practice.
                        # If we don't XACK, messages will be re-delivered to this or other consumers
                        # in the group after some time if they appear "pending".
                        # Let's use XACK for good measure.
                        redis_client.xack(RAW_NRF_STREAM_NAME, CONSUMER_GROUP_NAME, message_id)
            else:
                # No new messages within the block timeout
                # print("ControlProcessor: No new NRF data.") # Can be noisy
                pass

        except redis.exceptions.RedisError as e:
            print(f"ControlProcessor: Redis error in main loop: {e}. Attempting to continue...")
            time.sleep(1) # Wait a bit before retrying
        except KeyboardInterrupt:
            print("\nControlProcessor: Shutting down by user request.")
            break
        except Exception as e:
            print(f"ControlProcessor: An unexpected error occurred in main_loop: {e}")
            time.sleep(1)


if __name__ == "__main__":
    if setup_redis_client():
        try:
            main_loop()
        except KeyboardInterrupt:
            print("ControlProcessor: Main loop terminated by user.")
        finally:
            print("ControlProcessor: Cleaning up...")
            # No specific cleanup for redis client here, connection pool handles it.
            print("ControlProcessor: Script finished.")
    else:
        print("ControlProcessor: Could not initialize Redis. Exiting.")
    sys.exit(0)