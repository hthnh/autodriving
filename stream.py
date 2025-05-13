import socket
import struct
import pickle
import cv2
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.configure("preview")
picam2.start()

# Set up socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", 8080))  # Listen on all interfaces
server_socket.listen(1)

print("? Waiting for connection...")
client_socket, addr = server_socket.accept()
print("? Connected to:", addr)

while True:
    frame = picam2.capture_array()
    # ? Flip the frame using OpenCV
    frame = cv2.flip(frame, -1)  # Flip both horizontally & vertically (Rotate 180)
    # Use cv2.flip(frame, 1) for horizontal flip
    # Use cv2.flip(frame, 0) for vertical flip

    # Convert frame to JPEG
    _, buffer = cv2.imencode(".jpg", frame)
    data = pickle.dumps(buffer)
    size = struct.pack(">L", len(data))  # Pack size as 4 bytes

    try:
        client_socket.sendall(size + data)  # Send size + frame
    except:
        break  # Stop on error

client_socket.close()
server_socket.close()
cv2.destroyAllWindows()
picam2.stop()
