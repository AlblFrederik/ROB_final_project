#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-11-9
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

# pip install opencv-contrib-python
from pathlib import Path


import cv2
import numpy as np


frame = cv2.imread(str(Path(__file__).parent / "img.png"))
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

# Detect markers and draw them
corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
cv2.aruco.drawDetectedMarkers(frame, corners, ids)

cv2.imshow("Image with markers", frame)
cv2.waitKey()

# Estimate SE3 pose of the marker
camera_matrix = np.array([[2.39770761e+02, 0.00000000e+00, -1.60636414e-01],
                    [0.00000000e+00, 2.40106492e+02, 1.12861516e-01],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) * 0.015
distortion = np.array([3.01662024e-02,  -8.12422159e-04,  -5.32767362e-03,  -2.91039031e-02, 7.87830930e-06])
for i in range(len(ids)):
    # rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.04, camera_matrix, distCoeffs=distortion)
    rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.04, camera_matrix, distCoeffs=distortion)
    # cv2.drawFrameAxes(frame, camera_matrix, distortion, rvec, tvec, 0.04)
    cv2.aruco.drawAxis(frame, camera_matrix, distortion, rvec, tvec, 0.04)

    print(str(ids[i]) + str(tvec))

cv2.imshow("Image with frames", frame)
cv2.waitKey()
