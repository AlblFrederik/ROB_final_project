import cv2
import numpy as np
from pathlib import Path

# Set the path to the directory containing calibration images
calibration_images_path = Path(__file__).parent
print(calibration_images_path)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0)
objp = np.zeros((6 * 9, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Read images and find markers
images = sorted(calibration_images_path.glob("*.png"))

camera_matrix = np.array(
    [
        [240.0, 0, 0],
        [0, 240, 0],
        [0, 0, 1],
    ])
distortion = np.zeros(5)

for fname in images:
    img = cv2.imread(str(fname))

    if img is None:
        print(f"Error loading image: {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

    # Find the markers
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)

    # If markers are found, add object points and image points
    if corners:
        # Use the first detected marker for simplicity, adjust as needed
        marker_corners = corners[0][0] if len(corners[0]) > 0 else None

        if marker_corners is not None:
            # Generate object points based on the number of detected corners
            objp_i = np.zeros((len(marker_corners), 3), np.float32)
            objp_i[:, :2] = objp[:len(marker_corners), :2]

            objpoints.append(objp[:len(marker_corners)])  # Use a subset of objp
            imgpoints.append(marker_corners)

            # Draw and display the markers
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            cv2.imshow('img', img)
            cv2.waitKey(500)  # You can adjust the waiting time to see the images

cv2.destroyAllWindows()

flags = (
    cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO |
    cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 |
    cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6
)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], camera_matrix, distortion, flags=flags)

# Print the calibration results
print("Camera Matrix:\n", mtx)
print("\nDistortion Coefficients:\n", dist)

# Save the calibration results for later use
np.savez("calibration.npz", mtx=mtx, dist=dist)
