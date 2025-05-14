import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

for id in range(5):  # In 10 marker ID từ 0–9
    marker = aruco.generateImageMarker(aruco_dict, id, 400)  # 300x300 pixels
    cv2.imwrite(f"aruco_id_{id}.png", marker)
