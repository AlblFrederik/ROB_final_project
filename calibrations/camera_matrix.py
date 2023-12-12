import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)

objp = np.zeros((18 * 12, 3), np.float32)
objp[:, :2] = np.mgrid[0:18, 0:12].T.reshape(-1, 2)*0.015

# objp = np.zeros((18*12, 3), np.float32)
# objp[:, 0] = np.tile(np.arange(0, 18*1.5, 1.5), 12)
# objp[:, 1] = np.tile(np.arange(0, 1.5*12, 1.5), 18)
# objp[:, 2] = 123.5
# objp[:, 0] += 0  # -18
# objp[:, 1] += 0


# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob('*.jpg')
# images = ['../img2.jpg']
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_RGBA2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (18, 12), None)
    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (18, 12), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
cv.destroyAllWindows()

flags = (
    cv.CALIB_USE_INTRINSIC_GUESS
)

camera_matrix = np.array(
    [
        [240.0, 0, 0],
        [0, 240, 0],
        [0, 0, 1],
    ])
distortion = np.zeros(5)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], cameraMatrix=camera_matrix,
                                                  distCoeffs=distortion, flags=flags)
print(ret)
print(mtx)
print(dist)
print(rvecs)
print(tvecs)

cam_mat = np.array([[2.39770761e+02, 0.00000000e+00, -1.60636414e-01],
                    [0.00000000e+00, 2.40106492e+02, 1.12861516e-01],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
