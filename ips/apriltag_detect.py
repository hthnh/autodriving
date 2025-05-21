#tag36h11 tag25h9 tag16h5 tagCircle21h7 tagCircle49h12

import cv2
import apriltag

# Initialize camera (0 = default webcam)
cap = cv2.VideoCapture(0)

# Check if camera opened successfully
if not cap.isOpened():
    print("[ERROR] Cannot open camera")
    exit()

# Setup the detector
options = apriltag.DetectorOptions(families="tag36h11 tag25h9 tag16h5 tagCircle21h7 tagCircle49h12")
detector = apriltag.Detector(options)

print("[INFO] Starting video stream...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Failed to grab frame")
        break

    # Convert to grayscale for AprilTag detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    results = detector.detect(gray)
    for r in results:
        # Get corners and center
        corners = r.corners.astype(int)
        center = tuple(r.center.astype(int))

        # Draw bounding box
        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        # Draw center
        cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # Display tag family
        tag_family = r.tag_family.decode("utf-8")
        cv2.putText(frame, tag_family, (corners[0][0], corners[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Show result
    cv2.imshow("AprilTag Detection", frame)

    # Quit with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
