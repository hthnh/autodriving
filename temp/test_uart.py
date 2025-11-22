# uart_sender.py
# Module duy nhất chịu trách nhiệm giao tiếp UART với Arduino Master.
# Nó đọc lệnh đã xử lý từ Redis, yêu cầu và phân tích dữ liệu cảm biến,
# áp dụng logic tránh vật cản, và chỉ gửi lệnh cuối cùng xuống Master khi cần.

import redis
import serial
import time
import sys
import threading
from queue import Queue
import re # Thư viện Biểu thức chính quy (Regular Expressions)

# --- Configuration ---
# Redis Configuration
REDIS_HOST = 'localhost'
REDIS_PORT = 6379
redis_client = None
PROCESSED_COMMAND_STREAM_NAME = 'vehicle:commands_processed' # Đọc từ đây
RAW_NRF_STREAM_NAME_TO_CLEAR = 'nrf:raw_data'               # Xóa stream này khi khởi động
UART_SENDER_CONSUMER_GROUP_NAME = 'uart_sender_group'
UART_SENDER_CONSUMER_NAME = 'sender_instance_1'

# UART (Serial) to Arduino Master Configuration
SERIAL_PORT_MASTER = '/dev/serial0'  # Chỉnh lại cổng UART của bạn nếu cần
BAUD_RATE_MASTER = 9600
ser_master = None

# Obstacle Avoidance Configuration
OBSTACLE_THRESHOLD_CM = 15.0 # Ngưỡng khoảng cách nguy hiểm (5cm)
NUM_SENSORS = 6
ULTRASONIC_REQUEST_INTERVAL = 0.5 # Yêu cầu dữ liệu mỗi 500ms

# --- Global State Variables (Thread-Safe) ---
# Mảng lưu trạng thái khoảng cách của các cảm biến, khởi tạo với giá trị an toàn
last_known_distances = [float('inf')] * NUM_SENSORS 
OBSTACLE_DETECTED = False # Biến trạng thái toàn cục cho vật cản
lock = threading.Lock() # Khóa để bảo vệ truy cập các biến chia sẻ trên
uart_response_queue = Queue() # Hàng đợi để luồng đọc và luồng chính giao tiếp

# Global state for last successfully sent commands
last_sent_motor_left_dir = -1
last_sent_motor_left_speed = -1
last_sent_motor_right_dir = -1
last_sent_motor_right_speed = -1
last_sent_servo1_angle = -1
last_sent_servo2_angle = -1




# --- DEBUG STATS ---
# Dictionary chia sẻ giữa các luồng để lưu trữ thông số debug
debug_stats = {
    'main_loop_count': 0,
    'main_loop_proc_time_sum': 0.0,
    'main_loop_cycle_time_sum': 0.0,
    'reader_thread_lines_read': 0
}
DEBUG_PRINT_INTERVAL = 1.0 # In thông số debug mỗi 2 giây





# --- Setup Functions ---
def setup_redis_client():
    global redis_client
    try:
        print(f"UART_Sender: Connecting to Redis at {REDIS_HOST}:{REDIS_PORT}...")
        redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=False)
        redis_client.ping()
        print("UART_Sender: Redis connection established.")

        # --- Xóa các stream cũ khi khởi động ---
        streams_to_clear = [RAW_NRF_STREAM_NAME_TO_CLEAR, PROCESSED_COMMAND_STREAM_NAME]
        for stream_name in streams_to_clear:
            try:
                print(f"UART_Sender: Clearing Redis Stream '{stream_name}'...")
                result = redis_client.delete(stream_name)
                if result > 0: print(f"UART_Sender: Stream '{stream_name}' cleared.")
                else: print(f"UART_Sender: Stream '{stream_name}' did not exist.")
            except Exception as e_del: print(f"UART_Sender: Error clearing stream '{stream_name}': {e_del}")
        
        # Tạo consumer group cho stream mà script này sẽ đọc
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
            else: raise e_group
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
        time.sleep(2) # Chờ Arduino reset
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

# --- Thread-Safe UART Communication ---
def read_from_master_thread(ser, q, stats_dict, lock_obj):
    """Luồng nền để đọc liên tục từ cổng UART và đưa vào hàng đợi."""
    while True:
        try:
            if ser and ser.is_open and ser.in_waiting > 0:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    q.put(response)
                    # --- DEBUG STATS ---
                    with lock_obj:
                        stats_dict['reader_thread_lines_read'] += 1
            time.sleep(0.01)
        except serial.SerialException as e:
            print(f"UART_Reader_Thread: Serial error: {e}. Thread stopping.")
            break
        except Exception as e:
            print(f"UART_Reader_Thread: Unexpected error: {e}")
            time.sleep(1)

def send_uart_command(command_string):
    """Gửi lệnh xuống Arduino Master qua UART."""
    if ser_master and ser_master.is_open:
        try:
            # print(f"UART_Sender: SENDING -> {command_string}") # Bỏ comment để debug
            ser_master.write(command_string.encode('utf-8') + b'\n')
            time.sleep(0.07) # Thêm một khoảng nghỉ nhỏ sau mỗi lần gửi để Master có thời gian xử lý
        except Exception as e:
            print(f"UART_Sender: Error writing to serial port: {e}")

# --- Logic xử lý chính ---
def process_uart_responses():
    """Xử lý các chuỗi phản hồi từ Arduino Master."""
    global OBSTACLE_DETECTED, last_known_distances
    
    sensor_pattern = re.compile(r"Sensor\s*(\d+):\s*(\d+)\s*cm")
    
    something_changed_in_distances = False
    try:
        while not uart_response_queue.empty():
            response = uart_response_queue.get_nowait()
            
            match = sensor_pattern.match(response)
            if match:
                sensor_index = int(match.group(1)) - 1
                distance = int(match.group(2))
                
                if 0 <= sensor_index < NUM_SENSORS:
                    if last_known_distances[sensor_index] != distance:
                        last_known_distances[sensor_index] = distance
                        something_changed_in_distances = True
        
        if something_changed_in_distances:
            found_obstacle = any(0 < dist < OBSTACLE_THRESHOLD_CM for dist in last_known_distances)
            
            with lock:
                if found_obstacle != OBSTACLE_DETECTED:
                    OBSTACLE_DETECTED = found_obstacle
                    if OBSTACLE_DETECTED:
                        # print(f"UART_Sender: OBSTACLE DETECTED! Distances: {[f'{d:.0f}' for d in last_known_distances]}")
                        pass
                    else:
                        # print(f"UART_Sender: Obstacle cleared. Distances: {[f'{d:.0f}' for d in last_known_distances]}")
                        pass
    except Exception as e:
        print(f"UART_Sender: Error processing UART responses: {e}")

def main_loop():
    """Vòng lặp chính: yêu cầu cảm biến, đọc lệnh Redis, áp dụng logic và gửi UART."""
    global last_sent_motor_left_dir, last_sent_motor_left_speed, last_sent_motor_right_dir, \
           last_sent_motor_right_speed, last_sent_servo1_angle, last_sent_servo2_angle
    global debug_stats
    
    if not redis_client or not (ser_master and ser_master.is_open):
        print("UART_Sender: Redis or Serial not initialized. Exiting.")
        return

    reader_thread = threading.Thread(target=read_from_master_thread, args=(ser_master, uart_response_queue, debug_stats, lock), daemon=True)
    reader_thread.start()
    print("UART_Sender: UART reader thread started.")
    
    print(f"UART_Sender: Listening to Redis Stream '{PROCESSED_COMMAND_STREAM_NAME}'...")
    stream_to_read = {PROCESSED_COMMAND_STREAM_NAME: '>'}
    
    last_ultrasonic_request_time = time.monotonic()
    
    # --- DEBUG STATS ---
    last_debug_print_time = time.monotonic()
    last_main_loop_start_time = time.monotonic()

    while True:
        try:
            current_time = time.monotonic()
            loop_start_time = time.monotonic()
            cycle_time = loop_start_time - last_main_loop_start_time
            last_main_loop_start_time = loop_start_time

            
            # 1. Yêu cầu dữ liệu siêu âm định kỳ
            if current_time - last_ultrasonic_request_time >= ULTRASONIC_REQUEST_INTERVAL:
                send_uart_command("8:REQ_DATA")
                last_ultrasonic_request_time = current_time

            # 2. Xử lý các phản hồi từ Master (nếu có)
            process_uart_responses()

            # 3. Đọc và xử lý lệnh điều khiển từ Redis (non-blocking)
            messages = redis_client.xreadgroup(
                groupname=UART_SENDER_CONSUMER_GROUP_NAME,
                consumername=UART_SENDER_CONSUMER_NAME,
                streams=stream_to_read,
                count=1,
                block=1000 # Chờ tối đa 10ms
            )
            
            if messages:
                for _, message_list in messages:
                    for message_id, command_data_bytes in message_list:
                        new_m_left_dir = int(command_data_bytes.get(b'motor_left_dir', b'0'))
                        new_m_left_speed = int(command_data_bytes.get(b'motor_left_speed', b'0'))
                        new_m_right_dir = int(command_data_bytes.get(b'motor_right_dir', b'0'))
                        new_m_right_speed = int(command_data_bytes.get(b'motor_right_speed', b'0'))
                        new_s1_angle = int(command_data_bytes.get(b'servo1_angle', b'90'))
                        new_s2_angle = int(command_data_bytes.get(b'servo2_angle', b'90'))
                        
                        # 4. Áp dụng logic chặn vật cản
                        # Logic đơn giản: nếu có vật cản phía trước, không cho đi tới.
                        # Bạn có thể làm phức tạp hơn: ví dụ, cảm biến 0,1,2 là phía trước, 3,4,5 là phía sau.
                        is_moving_forward = (new_m_left_dir == 1 and new_m_left_speed > 0) or \
                                            (new_m_right_dir == 1 and new_m_right_speed > 0)
                        
                        with lock:
                            obstacle_is_present = OBSTACLE_DETECTED
                        
                        if is_moving_forward and obstacle_is_present:
                            # Ghi đè lệnh di chuyển bằng lệnh dừng
                            new_m_left_dir, new_m_left_speed = 0, 0
                            new_m_right_dir, new_m_right_speed = 0, 0
                        
                        # 5. Gửi lệnh cuối cùng xuống Master nếu có thay đổi
                        if new_m_left_dir != last_sent_motor_left_dir or new_m_left_speed != last_sent_motor_left_speed:
                            send_uart_command(f"9:MOTOR:0:{new_m_left_dir}:{new_m_left_speed}")
                            last_sent_motor_left_dir, last_sent_motor_left_speed = new_m_left_dir, new_m_left_speed

                        if new_m_right_dir != last_sent_motor_right_dir or new_m_right_speed != last_sent_motor_right_speed:
                            send_uart_command(f"9:MOTOR:1:{new_m_right_dir}:{new_m_right_speed}")
                            last_sent_motor_right_dir, last_sent_motor_right_speed = new_m_right_dir, new_m_right_speed

                        if new_s1_angle != last_sent_servo1_angle:
                            send_uart_command(f"9:SERVO:0:{new_s1_angle}")
                            last_sent_servo1_angle = new_s1_angle

                        if new_s2_angle != last_sent_servo2_angle:
                            send_uart_command(f"9:SERVO:1:{new_s2_angle}")
                            last_sent_servo2_angle = new_s2_angle
                        
                        # Xác nhận đã xử lý message với Redis
                        redis_client.xack(PROCESSED_COMMAND_STREAM_NAME, UART_SENDER_CONSUMER_GROUP_NAME, message_id)
            
            # --- DEBUG STATS ---
            # Đo lường thời gian xử lý và cập nhật thống kê
            processing_time = time.monotonic() - loop_start_time
            with lock:
                debug_stats['main_loop_count'] += 1
                debug_stats['main_loop_proc_time_sum'] += processing_time
                debug_stats['main_loop_cycle_time_sum'] += cycle_time
            
            # In báo cáo debug định kỳ
            if loop_start_time - last_debug_print_time >= DEBUG_PRINT_INTERVAL:
                with lock:
                    # Sao chép và reset stats để tránh giữ lock quá lâu
                    main_loops = debug_stats['main_loop_count']
                    proc_time_sum = debug_stats['main_loop_proc_time_sum']
                    cycle_time_sum = debug_stats['main_loop_cycle_time_sum']
                    lines_read = debug_stats['reader_thread_lines_read']
                    
                    debug_stats['main_loop_count'] = 0
                    debug_stats['main_loop_proc_time_sum'] = 0.0
                    debug_stats['main_loop_cycle_time_sum'] = 0.0
                    debug_stats['reader_thread_lines_read'] = 0
                
                # Tính toán và in kết quả
                if main_loops > 0:
                    main_loop_freq = main_loops / DEBUG_PRINT_INTERVAL
                    avg_proc_time_ms = (proc_time_sum / main_loops) * 1000
                    avg_cycle_time_ms = (cycle_time_sum / main_loops) * 1000
                    reader_freq = lines_read / DEBUG_PRINT_INTERVAL

                    print("-" * 30)
                    print(f"PERFORMANCE STATS (last {DEBUG_PRINT_INTERVAL}s):")
                    print(f"  Main Loop Freq:     {main_loop_freq:.2f} Hz")
                    print(f"  Avg Cycle Time:     {avg_cycle_time_ms:.2f} ms")
                    print(f"  Avg Processing Time:{avg_proc_time_ms:.2f} ms")
                    print(f"  Reader Thread Rate: {reader_freq:.2f} lines/sec")
                    print(last_known_distances)
                    print("-" * 30)
                
                last_debug_print_time = loop_start_time

            
            # Sleep nhỏ để vòng lặp chính không chiếm hết CPU
            time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nUART_Sender: Shutting down by user request.")
            break
        except Exception as e:
            print(f"UART_Sender: An unexpected error occurred in main_loop: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(1)

if __name__ == "__main__":
    if setup_redis_client() and setup_serial_to_master():
        try:
            send_uart_command("8:START")
            main_loop()
        finally:
            print("UART_Sender: Cleaning up...")
            if 'ser_master' in locals() and ser_master and ser_master.is_open:
                print("UART_Sender: Sending final stop command to motors before exit.")
                send_uart_command("9:MOTOR:0:0:0")
                time.sleep(0.01)

                send_uart_command("9:MOTOR:1:0:0")
                time.sleep(0.01)

                ser_master.close()
                print("UART_Sender: Serial port to Master closed.")
            print("UART_Sender: Script finished.")
    else:
        print("UART_Sender: Could not initialize Redis or Serial. Exiting.")
    
    sys.exit(0)