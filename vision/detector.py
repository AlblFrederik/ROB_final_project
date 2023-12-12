import cv2
import numpy as np


class Detector:
    @staticmethod
    def get_all(img, visualize=True):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

        # Detect markers and draw them
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
        cv2.aruco.drawDetectedMarkers(img, corners, ids)

        camera_matrix = np.array([[1.62889514e+03, 0.00000000e+00, 6.31677588e+02],
                                  [0.00000000e+00, 1.63545825e+03, 4.96521394e+02],
                                  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        distortion = np.array([3.01662024e-02, -8.12422159e-04, -5.32767362e-03, -2.91039031e-02, 7.87830930e-06])
        ret = []
        for i in range(len(ids)):
            # rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.04, camera_matrix, distCoeffs=distortion)
            rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.04, camera_matrix, distCoeffs=distortion)
            # cv2.drawFrameAxes(frame, camera_matrix, distortion, rvec, tvec, 0.04)
            cv2.aruco.drawAxis(img, camera_matrix, distortion, rvec, tvec, 0.04)
            ret.append([ids[i], rvec, tvec])

        if visualize:
            cv2.imshow("Image with frames", img)
            cv2.waitKey()
        return ret
