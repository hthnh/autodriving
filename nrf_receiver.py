# nrf_receiver.py
# Reads data from NRF24L01 and publishes it to a Redis Stream.

from RF24 import RF24, RF24_PA_LOW, RF24_250KBPS  # RF24_250KBPS might be needed based on your sender
import struct
import time
import redis
import sys

# --- NRF24L01 Configuration ---
# CE, CSN pins for Raspberry Pi. Adjust if your wiring is different.
# RPi BCM pins: CE=GPIO22 (corresponds to `radio = RF24(22, 0)` for older RPi library versions)
# or CE=GPIO25 if using spidev0.0 for CSN. The RF24 library constructor usually takes (CE, CSN_device_index)
# For Pi 5 and common libraries: radio = RF24(22, 0) means CE=GPIO22, CSN=SPI_CE0 (GPIO8)
# User previously had: radio = RF24(25, 0) -> CE=GPIO25, CSN=SPI_CE0 (GPIO8)
# Let's stick to user's previous CE pin, assuming CSN is managed by SPI CE0.
CE_PIN = 25 # GPIO25
CSN_PIN_INDEX = 0 # Using /dev/spidev0.0, so CSN index is 0

radio = RF24(CE_PIN, CSN_PIN_INDEX)

# Data structure: 11 bools + 4 int16 (joyX1, joyY1, joyX2, joyY2)
# '<' for little-endian, '?' for bool (1 byte), 'h' for short (int16, 2 bytes)
DATA_FORMAT = '<' + ('?' * 11) + ('h' * 4)
DATA_SIZE = struct.calcsize(DATA_FORMAT)

# NRF24L01 Pipe Address (must match the sender's transmitting address)
PIPE_ADDRESS = b'21533' # Example address, ensure it matches your sender

# --- Redis Configuration ---
REDIS_HOST = 'localhost'
REDIS_PORT = 6379
redis_client = None
RAW_NRF_STREAM_NAME = 'nrf:raw_data' # Stream to publish raw NRF data

def setup_radio():
    """Initializes and configures the NRF24L01 module."""

    

    if not radio.begin():
        raise RuntimeError("NRF24L01 hardware is not responding. Check wiring and CE/CSN pins.")
    time.sleep(0.5)
    radio.powerUp()
    # Set NRF24L01 parameters (must match sender)
    radio.setChannel(60)             # Example channel
    radio.setPALevel(RF24_PA_LOW)     # Power Amplifier level
    radio.setDataRate(RF24_250KBPS) # Example data rate, use RF24_1MBPS or RF24_2MBPS if sender uses that

    radio.openReadingPipe(1, PIPE_ADDRESS) # Open pipe 1 for listening
    radio.startListening()                 # Start listening for data
    
    print("NRF24L01 Receiver is ready and listening...")
    print("Listening on pipe address:", PIPE_ADDRESS)
    radio.printDetails() # Useful for debugging NRF setup

def setup_redis_client():
    """Initializes the connection to the Redis server."""
    global redis_client
    try:
        print(f"Connecting to Redis at {REDIS_HOST}:{REDIS_PORT}...")
        redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        redis_client.ping() # Check connection
        print("Redis connection established.")
        return True
    except redis.exceptions.ConnectionError as e:
        print(f"Failed to connect to Redis: {e}")
        print("Please ensure Redis server is running.")
        redis_client = None
        return False
    except Exception as e:
        print(f"An unexpected error occurred during Redis setup: {e}")
        redis_client = None
        return False

def main_loop():
    """Continuously listens for NRF data and publishes it to Redis."""
    if not redis_client:
        print("Redis client not initialized. Exiting.")
        return

    last_print_time = time.time()
    payload_counter = 0

    while True:
        if radio.available():
            buffer = radio.read(DATA_SIZE)
            
            if len(buffer) == DATA_SIZE:
                try:
                    # Unpack the received data
                    unpacked_data = struct.unpack(DATA_FORMAT, buffer)
                    
                    buttons_tuple = unpacked_data[:11]
                    joy_values_tuple = unpacked_data[11:]

                    # Prepare data for Redis Stream
                    # XADD requires a dictionary of field-value pairs
                    redis_payload = {
                        f'btn{i}': int(buttons_tuple[i]) for i in range(11) # Convert bool to int (0 or 1)
                    }
                    redis_payload['joyX1'] = joy_values_tuple[0]
                    redis_payload['joyY1'] = joy_values_tuple[1]
                    redis_payload['joyX2'] = joy_values_tuple[2]
                    redis_payload['joyY2'] = joy_values_tuple[3]
                    redis_payload['timestamp'] = time.time() # Add a server-side timestamp
                    print(f"DEBUG NRF_Receiver: Publishing to Redis: {redis_payload}") # <--- THÊM DÒNG NÀY

                    # Publish to Redis Stream
                    message_id = redis_client.xadd(RAW_NRF_STREAM_NAME, redis_payload)
                    payload_counter +=1

                    # Optional: Print received data periodically for debugging
                    current_time = time.time()
                    if current_time - last_print_time >= 1.0: # Print every 1 second
                        print(f"[{time.strftime('%H:%M:%S')}] Msg ID: {message_id} | Payloads sent in last sec: ~{payload_counter}")
                        print(f"  Buttons: {buttons_tuple}")
                        print(f"  Joy1(X,Y): {joy_values_tuple[0]}, {joy_values_tuple[1]} | Joy2(X,Y): {joy_values_tuple[2]}, {joy_values_tuple[3]}")
                        last_print_time = current_time
                        payload_counter = 0

                except struct.error as e:
                    print(f"Error unpacking NRF data: {e}. Buffer length: {len(buffer)}")
                except redis.exceptions.RedisError as e:
                    print(f"Redis error when publishing: {e}")
                    # Consider attempting to reconnect to Redis here if connection drops
                except Exception as e:
                    print(f"An error occurred in main loop: {e}")
            else:
                print(f"Warning: Received NRF buffer size mismatch. Expected {DATA_SIZE}, got {len(buffer)}")
        
        time.sleep(0.001) # Small sleep to prevent high CPU usage if NRF data is sparse

if __name__ == "__main__":
    try:
        print("Starting NRF24L01 Receiver Script...")
        setup_radio()
        if setup_redis_client():
            main_loop()
        else:
            print("Could not connect to Redis. Exiting.")
            
    except RuntimeError as e:
        print(f"NRF24L01 Runtime Error: {e}")
        print("This often means the radio hardware is not connected properly or CE/CSN pin configuration is incorrect.")
    except KeyboardInterrupt:
        print("\nNRF Receiver stopped by user.")
    except Exception as e:
        print(f"An unexpected critical error occurred: {e}")
    finally:
        if 'radio' in locals() and radio: # Check if radio object was initialized
            try:
                radio.stopListening()
                print("NRF24L01 radio stopped listening.")
            except Exception as e_radio_stop:
                print(f"Error stopping NRF radio: {e_radio_stop}")
        
        if redis_client:
            try:
                # Optional: You might want to indicate that the NRF source has stopped
                # redis_client.xadd(RAW_NRF_STREAM_NAME, {'status': 'stopped', 'timestamp': time.time()})
                print("NRF Receiver Redis client will not be explicitly closed here, as it's a publisher.")
            except Exception as e_redis_close:
                print(f"Error during Redis cleanup: {e_redis_close}")

        print("NRF Receiver script finished.")
        sys.exit(0)