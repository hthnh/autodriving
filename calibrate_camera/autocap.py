from picamera2 import Picamera2
import time
import os
import cv2

# ==== C?u hnh ====
output_folder = "calib_images"
capture_interval = 2  # Giy gi?a m?i l?n ch?p
max_images = 20       # S? ?nh c?n ch?p

# ==== T?o th? m?c n?u ch?a c====
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (1280, 720)})
picam2.configure(preview_config)
picam2.start()

time.sleep(5)
count = 0
last_capture_time = time.time()


while count < max_images:
    frame = picam2.capture_array()

    # Hi?n th? ?nh trn c?a s?
    cv2.imshow("Calibration Capture", frame)

    # T? ??ng l?u ?nh m?i capture_interval giy
    if time.time() - last_capture_time >= capture_interval:
        filename = os.path.join(output_folder, f"image_{count:02d}.jpg")
        cv2.imwrite(filename, frame)
        print(f"? ? l?u {filename}")
        count += 1
        last_capture_time = time.time()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
print("? K?t thc ch??ng tr.")
