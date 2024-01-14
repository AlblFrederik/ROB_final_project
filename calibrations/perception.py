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
from pathlib import Path
from os import listdir

ALPHA = 2  # Contrast Control
BETA = 10  # Brightness Control
CHESSBOARD_SIZE = (18, 12)  # (25, 17) # Number of chessboard internal corners
CHESSBOARD_SQUARE_SIZE = 0.015  # 0.01 #m
CALIBRATION_FOLDER = "images"


def camera_calibration():
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = CHESSBOARD_SQUARE_SIZE * np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    # print(objp)

    obj_points = []
    img_points = []
    ret = None

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
        # termination criteria
        if ret == True:
            obj_points.append(objp)
            corners2 = cv2.cornerSubPix(adjusted_gray, corners, winSize=(11, 11), zeroZone=(-1, -1), criteria=criteria)
            img_points.append(corners2)
            # Draw and display the corners
            cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, corners2, ret)
            cv2.imwrite(folder_dir + '/' + 'marked_' + frame_name, frame)

    try:
        os.remove(os.getcwd() + "/adjusted.png")
    except Exception as E:
        print("adjusted.png deletion: {}".format(E))
    # Camera calibration:
    _, K, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape, None, None)

    # Distortion defect compensation
    K_new, roi = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, gray.shape, 1, gray.shape)
    return ret, K_new, K


if __name__ == '__main__':
    ret, K_new, K = camera_calibration()
    print(K, "\n")
    print(K_new)
