import cv2
import cv2.aruco as aruco

# Chọn dictionary bạn đã dùng để in marker
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Tham số phát hiện (phiên bản mới không cần .create())
parameters = aruco.DetectorParameters()

# Khởi tạo detector
detector = aruco.ArucoDetector(aruco_dict, parameters)

# Mở camera (0 là webcam mặc định)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

while True:
    # ret, frame = cap.read()
    # if not ret:
    #     print("❌ Failed to grab frame")
    #     break
    frame = cv2.imread("aruco_id_0.png")
    


    # Phát hiện marker
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        print("✅ Detected IDs:", ids.flatten())
    else:
        print("🔍 No marker detected")

    # Hiển thị
    cv2.imshow("ArUco Detection", frame)

    # Nhấn ESC để thoát
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
