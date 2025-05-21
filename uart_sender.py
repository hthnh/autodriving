# uart_sender.py
# Reads processed vehicle commands from a Redis Stream,
# sends them to the Arduino Master via UART ONLY IF THEY HAVE CHANGED.

import redis
import serial
import time
import sys
# import json # Not strictly needed if parsing individual fields

# --- Redis Configuration ---
REDIS_HOST = 'localhost'
REDIS_PORT = 6379
redis_client = None

PROCESSED_COMMAND_STREAM_NAME = 'vehicle:commands_processed'
UART_SENDER_CONSUMER_GROUP_NAME = 'uart_sender_group'
UART_SENDER_CONSUMER_NAME = 'sender_instance_1'

# --- UART (Serial) to Arduino Master Configuration ---
SERIAL_PORT_MASTER = '/dev/serial0'
BAUD_RATE_MASTER = 9600
ser_master = None

# --- Global state for last successfully sent commands to Arduino Master ---
# Initialize to values that will likely differ from the first valid command,
# ensuring the first set of commands gets through.
# Or, initialize to a known "neutral" or "stopped" state.
last_sent_motor_left_dir = -1   # Assuming valid dirs are 0, 1, 2
last_sent_motor_left_speed = -1 # Assuming valid speeds are 0-255
last_sent_motor_right_dir = -1
last_sent_motor_right_speed = -1
last_sent_servo1_angle = -1     # Assuming valid angles are 0-180
last_sent_servo2_angle = -1

def setup_redis_client():
    global redis_client
    try:
        print(f"UART_Sender: Connecting to Redis at {REDIS_HOST}:{REDIS_PORT}...")
        redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=False)
        redis_client.ping()
        print("UART_Sender: Redis connection established.")

        # --- Optional: Clearing old stream data (as per previous request) ---
        streams_to_clear = ['nrf:raw_data', PROCESSED_COMMAND_STREAM_NAME]
        for stream_name in streams_to_clear:
            try:
                print(f"UART_Sender: Attempting to clear Redis Stream '{stream_name}'...")
                result = redis_client.delete(stream_name)
                if result > 0: print(f"UART_Sender: Stream '{stream_name}' cleared.")
                else: print(f"UART_Sender: Stream '{stream_name}' did not exist or was empty.")
            except Exception as e_del: print(f"UART_Sender: Error clearing stream '{stream_name}': {e_del}")
        # --- End Clearing ---

        try:
            redis_client.xgroup_create(
                name=PROCESSED_COMMAND_STREAM_NAME,
                groupname=UART_SENDER_CONSUMER_GROUP_NAME,
                id='0',
                mkstream=True
            )
            print(f"UART_Sender: Consumer group '{UART_SENDER_CONSUMER_GROUP_NAME}' ensured for '{PROCESSED_COMMAND_STREAM_NAME}'.")
        except redis.exceptions.ResponseError as e_group:
            if "BUSYGROUP" in str(e_group):
                print(f"UART_Sender: Consumer group '{UART_SENDER_CONSUMER_GROUP_NAME}' already exists.")
            else:
                print(f"UART_Sender: Redis error (group for '{PROCESSED_COMMAND_STREAM_NAME}'): {e_group}")
                return False
        return True
    except Exception as e:
        print(f"UART_Sender: Failed to connect/setup Redis: {e}")
        redis_client = None
        return False

def setup_serial_to_master():
    global ser_master
    try:
        print(f"UART_Sender: Connecting to Arduino Master on {SERIAL_PORT_MASTER}...")
        ser_master = serial.Serial(SERIAL_PORT_MASTER, BAUD_RATE_MASTER, timeout=0.1)
        time.sleep(2)
        if ser_master.is_open:
            print("UART_Sender: Serial connection to Master established.")
            return True
        else:
            print("UART_Sender: Serial port to Master could not be opened.")
            ser_master = None
            return False
    except Exception as e:
        print(f"UART_Sender: Failed to connect to Arduino Master on {SERIAL_PORT_MASTER}: {e}")
        ser_master = None
        return False

def send_uart_command(command_string):
    if ser_master and ser_master.is_open:
        try:
            # print(f"UART_Sender: Sending to Master: {command_string}") # Debug
            ser_master.write(command_string.encode('utf-8') + b'\n')
            time.sleep(0.05)
        except Exception as e:
            print(f"UART_Sender: Error writing to serial port {SERIAL_PORT_MASTER}: {e}")

def main_loop():
    global last_sent_motor_left_dir, last_sent_motor_left_speed
    global last_sent_motor_right_dir, last_sent_motor_right_speed
    global last_sent_servo1_angle, last_sent_servo2_angle
    
    if not redis_client or not (ser_master and ser_master.is_open):
        print("UART_Sender: Redis or Serial not initialized. Exiting.")
        return

    print(f"UART_Sender: Listening to Redis Stream '{PROCESSED_COMMAND_STREAM_NAME}'...")
    stream_to_read = {PROCESSED_COMMAND_STREAM_NAME: '>'}

    while True:
        try:
            messages = redis_client.xreadgroup(
                groupname=UART_SENDER_CONSUMER_GROUP_NAME,
                consumername=UART_SENDER_CONSUMER_NAME,
                streams=stream_to_read,
                count=1,
                block=1000
            )

            if messages:
                for stream_name, message_list in messages:
                    for message_id, command_data_bytes in message_list:
                        # print(f"UART_Sender: Received from Redis (ID: {message_id.decode()}): {command_data_bytes}") # Debug
                        any_command_sent_this_cycle = False
                        try:
                            # Extract command components
                            new_m_left_dir = int(command_data_bytes.get(b'motor_left_dir', b'0'))
                            new_m_left_speed = int(command_data_bytes.get(b'motor_left_speed', b'0'))
                            new_m_right_dir = int(command_data_bytes.get(b'motor_right_dir', b'0'))
                            new_m_right_speed = int(command_data_bytes.get(b'motor_right_speed', b'0'))
                            new_s1_angle = int(command_data_bytes.get(b'servo1_angle', b'90'))
                            new_s2_angle = int(command_data_bytes.get(b'servo2_angle', b'90'))

                            # --- Motor Left Command ---
                            if new_m_left_dir != last_sent_motor_left_dir or \
                               new_m_left_speed != last_sent_motor_left_speed:
                                cmd_left_motor = f"9:MOTOR:0:{new_m_left_dir}:{new_m_left_speed}"
                                send_uart_command(cmd_left_motor)
                                last_sent_motor_left_dir = new_m_left_dir
                                last_sent_motor_left_speed = new_m_left_speed
                                any_command_sent_this_cycle = True
                                # print(f"UART_Sender: Updated Left Motor: Dir={new_m_left_dir}, Spd={new_m_left_speed}")


                            # --- Motor Right Command ---
                            if new_m_right_dir != last_sent_motor_right_dir or \
                               new_m_right_speed != last_sent_motor_right_speed:
                                cmd_right_motor = f"9:MOTOR:1:{new_m_right_dir}:{new_m_right_speed}"
                                # Optional: small delay if multiple commands sent in one cycle
                                # if any_command_sent_this_cycle: time.sleep(0.005) 
                                send_uart_command(cmd_right_motor)
                                last_sent_motor_right_dir = new_m_right_dir
                                last_sent_motor_right_speed = new_m_right_speed
                                any_command_sent_this_cycle = True
                                # print(f"UART_Sender: Updated Right Motor: Dir={new_m_right_dir}, Spd={new_m_right_speed}")

                            # --- Servo 1 Command ---
                            if new_s1_angle != last_sent_servo1_angle:
                                cmd_servo1 = f"9:SERVO:0:{new_s1_angle}"
                                # if any_command_sent_this_cycle: time.sleep(0.005)
                                send_uart_command(cmd_servo1)
                                last_sent_servo1_angle = new_s1_angle
                                any_command_sent_this_cycle = True
                                # print(f"UART_Sender: Updated Servo 1: Angle={new_s1_angle}")
                                
                            # --- Servo 2 Command ---
                            if new_s2_angle != last_sent_servo2_angle:
                                cmd_servo2 = f"9:SERVO:1:{new_s2_angle}"
                                # if any_command_sent_this_cycle: time.sleep(0.005)
                                send_uart_command(cmd_servo2)
                                last_sent_servo2_angle = new_s2_angle
                                # any_command_sent_this_cycle = True # No command follows this one in this block
                                # print(f"UART_Sender: Updated Servo 2: Angle={new_s2_angle}")
                            
                            # Acknowledge message from Redis
                            redis_client.xack(PROCESSED_COMMAND_STREAM_NAME, UART_SENDER_CONSUMER_GROUP_NAME, message_id)

                        except ValueError as e:
                            print(f"UART_Sender: Error parsing command data from Redis: {e}. Data: {command_data_bytes}")
                        except Exception as e:
                            print(f"UART_Sender: Error processing message {message_id.decode()}: {e}")
            # else:
                # print("UART_Sender: No new processed commands in Redis.") # Can be noisy

        except redis.exceptions.RedisError as e:
            print(f"UART_Sender: Redis error in main loop: {e}. Attempting to continue...")
            time.sleep(1) # Wait a bit before retrying Redis operations
        except KeyboardInterrupt:
            print("\nUART_Sender: Shutting down by user request.")
            break
        except Exception as e:
            print(f"UART_Sender: An unexpected error occurred in main_loop: {e}")
            time.sleep(1)


if __name__ == "__main__":
    redis_ok = setup_redis_client()
    serial_ok = setup_serial_to_master()

    if redis_ok and serial_ok:
        try:
            main_loop()
        except KeyboardInterrupt:
            print("UART_Sender: Main loop terminated by user.")
        finally:
            print("UART_Sender: Cleaning up...")
            if ser_master and ser_master.is_open:
                print("UART_Sender: Sending final stop command to motors before exit.")
                # Send stop regardless of last state for safety on exit
                send_uart_command("9:MOTOR:0:0:0") 
                time.sleep(0.01) # Give time for command to be sent/processed
                send_uart_command("9:MOTOR:1:0:0")
                time.sleep(0.01)
                ser_master.close()
                print("UART_Sender: Serial port to Master closed.")
            print("UART_Sender: Script finished.")
    else:
        print("UART_Sender: Could not initialize Redis or Serial. Exiting.")
    
    sys.exit(0)