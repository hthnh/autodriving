import socket
import struct
import pickle
import cv2

# Connect to the Raspberry Pi (replace with Pi's actual IP)
HOST = '192.168.10.188'  # ← Change this to your Pi's IP address
PORT = 8080             # Same port used in Pi code
cam_inx = 1
# Create socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT+cam_inx))

data = b""
payload_size = struct.calcsize(">L")

while True:
    # Receive message size (4 bytes)
    while len(data) < payload_size:
        packet = client_socket.recv(4096)
        if not packet:
            break
        data += packet

    if len(data) < payload_size:
        break

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]

    # Receive frame data
    while len(data) < msg_size:
        packet = client_socket.recv(4096)
        if not packet:
            break
        data += packet

    if len(data) < msg_size:
        break

    frame_data = data[:msg_size]
    data = data[msg_size:]

    # Deserialize frame
    frame = pickle.loads(frame_data)
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

    # Display frame
    cv2.imshow("Camera Stream from Pi", frame)
    if cv2.waitKey(1) == 27:  # Press Esc to exit
        break

client_socket.close()
cv2.destroyAllWindows()
