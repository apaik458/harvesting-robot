# A script to help capture the 21 photos used for camera calibration

import cv2

cap = cv2.VideoCapture(4)
count = 0

while True:
    ret, frame = cap.read()
    cv2.imshow("Calibration capture - press S to save, Q to quit", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        filename = f"calib_{count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved {filename}")
        count += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()