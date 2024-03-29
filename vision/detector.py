import cv2
import numpy as np
from vision import Camera


class Detector:

    def __init__(self, visualize=True):
        self.visualize = visualize

    def get_all(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

        # Detect markers and draw them
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        # print(f"corners{corners}")

        camera_matrix = np.array([[1.62889514e+03, 0.00000000e+00, 6.31677588e+02],
                                  [0.00000000e+00, 1.63545825e+03, 4.96521394e+02],
                                  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        distortion = np.array([3.01662024e-02, -8.12422159e-04, -5.32767362e-03, -2.91039031e-02, 7.87830930e-06])
        ret = []
        ret_ids = []
        if ids is not None:
            if len(ids) > 1:
                ids = ids.squeeze()
            else:
                ids = ids[0]
            for i in range(len(ids)):
                rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.04, camera_matrix, distCoeffs=distortion)
                # cv2.drawFrameAxes(frame, camera_matrix, distortion, rvec, tvec, 0.04)
                cv2.aruco.drawAxis(img, camera_matrix, distortion, rvec, tvec, 0.04)
                ret_ids.append(ids[i])

                corners1 = corners[i]
                corners1 = corners1.squeeze()
                dx, dy = corners1[1] - corners1[0]
                angle = 180 / np.pi * np.arctan2(dy, dx)
                angle = angle % 90
                rvec = rvec.squeeze()
                rvec[2] = angle
                ret.append(np.concatenate((tvec.squeeze(), rvec.squeeze())))

        else:
            return [], []

        if self.visualize:
            cv2.imshow("Detected bricks", img)
            cv2.waitKey()
        return ret, ret_ids