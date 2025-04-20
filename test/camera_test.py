import cv2

# Settings
FLIP = -1  # Flip both horizontally and vertically

# Force V4L2 backend
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Give camera a chance to warm up
for _ in range(5):
    cap.read()

ret, frame = cap.read()
if ret:
    frame = cv2.flip(frame, FLIP)
    cv2.imwrite("test.jpg", frame)
else:
    print("Failed to read from camera.")

cap.release()   