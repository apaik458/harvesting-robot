# A script to obtain camera_matrix and dist_coeffs for ArUco marker detection

import cv2
import numpy as np
import glob

SQUARES_X = 7
SQUARES_Y = 10
SQUARE_SIZE_MM = 39.2
MARKER_SIZE_MM = 29.0  # roughly 75% of square size, adjust if needed

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_SIZE_MM, MARKER_SIZE_MM, aruco_dict)
detector = cv2.aruco.CharucoDetector(board)

all_corners = []
all_ids = []
image_size = None

images = glob.glob("calib_*.jpg")
print(f"Found {len(images)} images")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    image_size = gray.shape[::-1]

    charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)

    if charuco_ids is not None and len(charuco_ids) > 4:
        all_corners.append(charuco_corners)
        all_ids.append(charuco_ids)
        print(f"{fname}: found {len(charuco_ids)} corners")
    else:
        print(f"{fname}: not enough corners found, skipping")

ret, camera_matrix, dist_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
    all_corners, all_ids, board, image_size, None, None
)

print(f"\nCalibration error (lower is better): {ret:.4f}")
print(f"\ncamera_matrix = np.array({camera_matrix.tolist()})")
print(f"\ndist_coeffs = np.array({dist_coeffs.tolist()})")