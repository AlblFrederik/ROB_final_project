import os
import numpy as np
import cv2
from vision import *
from runner_utils import *


def homography_calibration():
    # get_homography_data()
    detect_and_save()
    cam, dkt = load_arrays_from_file()
    X_dkt, U_cam = prepare_data_homography(cam, dkt)
    H, _ = cv2.findHomography(U_cam, X_dkt)
    print(X_dkt, U_cam)
    print(H)
    xy_true = X_dkt[0,:]
    xy_calculated = H @ np.append(U_cam[0,:], [1])
    print(f"xy_true: {xy_true}, xy_calculated:{xy_calculated}")
    return H


def prepare_data_homography(cam_vecs, dkt_vecs):
    T_cam_list = []
    T_dkt_list = []
    # X = np.zeros(len(cam_vecs), 2)
    # U = np.zeros(len(cam_vecs), 2)
    X = []
    U = []
    for i in range(len(cam_vecs)):
        cam_vec = cam_vecs[i]
        cam_vec = np.array(cam_vec) * 1000
        dkt_vec = dkt_vecs[i]
        if cam_vec is not None and len(cam_vec) > 0:
            xy = dkt_vec[:2]
            uv = cam_vec[:2]
            X.append(xy)
            U.append(uv)
            # X[i] = xy
            # U[i] = uv
    return np.array(X), np.array(U)

def load_arrays_from_file():
    loaded_data = np.load('homography_data.npz')
    vec_1, vec_2 = [], []
    for i, key in enumerate(loaded_data.keys()):
        loaded_array = loaded_data[key]
        print(f"Loaded Array {i + 1}:")
        print(loaded_array)
        vec_1.append(loaded_array[0])
        vec_2.append(loaded_array[1])
    return vec_1, vec_2


def detect_and_save():
    detector = Detector(False)
    img_list = os.listdir("data_homography")
    array_to_save = []
    OK = 0
    for img in img_list:
        image = cv2.imread(f"./data_homography/{img}")
        str_name = img.split("z")[0].replace('[', '').replace(']', '')
        # print(img)

        dkt_data = np.array(np.fromstring(str_name, dtype=float, sep=','))
        # print(dkt_data)
        cam_data, ids = detector.get_all(image)
        if cam_data is not None and len(cam_data) > 0:
            array_to_save.append([cam_data[0], dkt_data])
    np.savez('homography_data.npz', *array_to_save)


def get_homography_data():
    OK = 0
    ERROR = 0
    tty_dev = r"/dev/ttyUSB0"
    camera = Camera(False)
    robot = Robot(tty_dev, False)
    X = np.linspace(300, 750, 9)
    Y = np.linspace(-300, 275, 8)
    z = 50
    for x in X:
        for y in Y:
            try:
                # xyz = np.array([x, y, z, 0, 90, 0]) + np.array([0, 0, 50, 0, 0, 0])
                # ret = robot.move_xyz(xyz)
                path = f"data_homography/{str([x, y, z, 0, 90, 0])}z.png"
                print(path)
                rot = np.random.randn() * 90
                ret = robot.lay_down_brick([x, y, z, 0, 90, rot])
                if ret:
                    robot.move_xyz([100, -400, 200, 0, 90, 0])  # move away for photo
                    camera.save_img(path)                       # take photo
                    robot.pick_up_brick([x, y, z, 0, 90, rot])
                    OK += 1
            except:
                ERROR += 1
            print(f"SUM: {OK + ERROR}, OK{OK}, ERROR{ERROR}")


if __name__ == "__main__":
    homography_calibration()