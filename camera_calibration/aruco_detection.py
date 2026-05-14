import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

camera_matrix = np.array([
    [603.5139681293208,   0.0, 322.4871143178484],
    [  0.0, 606.2351953521359, 238.09114115221269],
    [  0.0,   0.0,   1.0]
], dtype=float)
dist_coeffs = np.array([[-0.005862800130731961, 1.193475628221078, 0.0006140022848452339, -0.0003950816468824019, -4.37080936712476]])
MARKER_SIZE_CM = 3.85

cap = cv2.VideoCapture(4)

while True:
    ret, frame = cap.read()
    corners, ids, rejected = detector.detectMarkers(frame)
    if ids is not None:
        for i in range(len(ids)):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i:i+1], MARKER_SIZE_CM, camera_matrix, dist_coeffs
            )
            x, y, z = tvec[0][0]
            y = -y  # Invert y-axis to match robot's coordinate system
            print(f"x={x:.2f}cm  y={y:.2f}cm  z={z:.2f}cm")
    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()