#!/usr/bin/env python
# Podle vzroru:
#   Copyright (c) CTU -- All Rights Reserved
#   Created on: 2023-10-23
#   Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

# pip install opencv-python
# pip install opencv-contrib-python (for aruco)
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
# from robotics_toolbox.core import SE3, SO3
from pathlib import Path
from os import listdir

ALPHA = 2  # Contrast Control
BETA = 10  # Brightness Control
CHESSBOARD_SIZE = (13, 9)  # (25, 17) # Number of chessboard internal corners
CHESSBOARD_SQUARE_SIZE = 0.018  # 0.01 #m
CALIBRATION_FOLDER = "images"

# Sources:
# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
"""
def hand_eye(a: list[SE3], b: list[SE3]) -> tuple[SE3, SE3]:
   	#Solve A^iX=YB^i, return X, Y

    rvec_a = [T.rotation.log() for T in a]
    tvec_a = [T.translation for T in a]
    rvec_b = [T.rotation.log() for T in b]
    tvec_b = [T.translation for T in b]

    Rx, tx, Ry, ty = cv2.calibrateRobotWorldHandEye(rvec_a, tvec_a, rvec_b, tvec_b)
    return SE3(tx[:, 0], SO3(Rx)), SE3(ty[:, 0], SO3(Ry))
"""


def calibrate_camera():
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = CHESSBOARD_SQUARE_SIZE * np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    # print(objp)

    obj_points = []
    img_points = []
    K = []

    folder_dir = str(Path(__file__).parent / CALIBRATION_FOLDER)
    for frame_name in os.listdir(folder_dir):
        if (frame_name.count('marked') > 0):
            print('STOP')
            continue

        frame = cv2.imread(folder_dir + '/' + frame_name)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        adjusted_gray = cv2.convertScaleAbs(gray, alpha=ALPHA, beta=BETA)
        cv2.imwrite('adjusted.png', adjusted_gray)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(adjusted_gray, CHESSBOARD_SIZE,
                                                 None)  # flags=cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK)
        print(ret)
        # termination criteria
        if ret == True:
            obj_points.append(objp)
            corners2 = cv2.cornerSubPix(adjusted_gray, corners, winSize=(11, 11), zeroZone=(-1, -1), criteria=criteria)
            img_points.append(corners2)
            # Draw and display the corners
            cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, corners2, ret)
            cv2.imwrite(folder_dir + '/' + 'marked_' + frame_name, frame)

    # Camera calibration:
    _, K, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape, None, None)

    # Distortion defect compensation
    K_new, roi = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, gray.shape, 1, gray.shape)
    return ret, K_new, K  # camera matrix


def detect_aruco(image_name, camera_matrix, time_limit=5000):
    frame = cv2.imread(str(Path(__file__).parent / image_name))  # image_name: "img.png"
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imwrite('img1.png', gray)

    adjusted = cv2.convertScaleAbs(gray, alpha=ALPHA, beta=BETA)
    cv2.imwrite('adjusted.png', adjusted)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict)

    # Detect markers and draw them
    (corners, ids, rejected) = detector.detectMarkers(adjusted)
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # cv2.imshow("Image with markers", frame)
    cv2.imwrite('img2.png', frame)
    # cv2.waitKey(time_limit)

    distortion = np.zeros(5)

    MARKER_LENGTH = 0.04
    for i in range(len(ids)):
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], MARKER_LENGTH, camera_matrix,
                                                            distCoeffs=distortion)
        cv2.drawFrameAxes(frame, camera_matrix, distortion, rvec, tvec, MARKER_LENGTH)
        # print(tvec)

    # cv2.imshow("Image with frames", frame)
    cv2.imwrite('img3.png', frame)
    # cv2.waitKey(time_limit)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # Estimate SE3 pose of the marker
    """
    camera_matrix = np.array(
        [
            [240.0, 0, 0],
            [0, 240, 0],
            [0, 0, 1],
        ]
    )
    """

    # detect_aruco("img.png", camera_matrix)
    ret, K_new, K = calibrate_camera()
    print(K, "\n")
    print(K_new)
