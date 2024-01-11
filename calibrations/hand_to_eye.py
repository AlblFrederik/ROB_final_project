#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-20
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
# pip install opencv-python
import numpy as np
from robotics_toolbox.core import SE3, SO3
import cv2
from vision import Detector
import os


def hand_eye(a, b):
    # def hand_eye(a: list[SE3], b: list[SE3]) -> tuple[SE3, SE3]:
    """Solve AX=ZB, return X, Z"""

    rvec_a = [T.rotation.log() for T in a]
    tvec_a = [T.translation for T in a]
    rvec_b = [T.rotation.log() for T in b]
    tvec_b = [T.translation for T in b]

    Rx, tx, Ry, ty = cv2.calibrateRobotWorldHandEye(rvec_a, tvec_a, rvec_b, tvec_b)
    return SE3(tx[:, 0], SO3(Rx)), SE3(ty[:, 0], SO3(Ry))


def tbc(cam_data, dkt_data):
    cam_t = cam_data[:3]
    cam_r = cam_data[3:] #np.flip(cam_data[3:], axis = 0)
    dkt_t = np.array(dkt_data[:3]) / 1000
    angle_x, angle_y, angle_z = np.array(dkt_data[3:])/180 * np.pi
    dkt_r = SO3.rx(angle_x) * SO3.ry(angle_y) * SO3.rz(angle_z)
    T_cam = SE3(cam_t, SO3.exp(cam_r))
    T_dkt = SE3(dkt_t, dkt_r)
    return T_cam, T_dkt


def detect_and_save():
    detector = Detector(False)
    img_list = os.listdir("data")
    array_to_save = []
    OK = 0
    for img in img_list:
        image = cv2.imread(f"./data/{img}")
        str_name = img.split("z")[0].replace('[', '').replace(']', '')
        # print(img)

        dkt_data = np.array(np.fromstring(str_name, dtype=float, sep=','))
        # print(dkt_data)
        cam_data, ids = detector.get_all(image)
        if cam_data is not None and len(cam_data) > 0:
            array_to_save.append([cam_data[0], dkt_data])
    np.savez('mult_arrays.npz', *array_to_save)

def detect_and_save_example():
    array_to_save = []
    for i in range(50):
        array_to_save.append([[1, 2, 1, 0, 0, 0], [1, 1, 2, 0, 0, 0]])
    np.savez('mult_arrays.npz', *array_to_save)

def load_arrays_from_file():
    loaded_data = np.load('mult_arrays.npz')
    vec_1, vec_2 = [], []
    for i, key in enumerate(loaded_data.keys()):
        loaded_array = loaded_data[key]
        print(f"Loaded Array {i + 1}:")
        print(loaded_array)
        vec_1.append(loaded_array[0])
        vec_2.append(loaded_array[1])
    return vec_1, vec_2


def prepare_data(cam_vecs, dkt_vecs):
    T_cam_list = []
    T_dkt_list = []
    for i in range(len(cam_vecs)):
        cam_vec = cam_vecs[i]
        dkt_vec = dkt_vecs[i]
        if cam_vec is not None and len(cam_vec) > 0:
            T_cam, T_dkt = tbc(cam_vec, dkt_vec)
            T_cam_list.append(T_cam)
            T_dkt_list.append(T_dkt)
    return T_cam_list, T_dkt_list

if __name__ == "__main__":
    np.random.seed(1)

    # gt_T_gt = SE3(translation=[0.05, 0.1, 0.2], rotation=SO3.exp([np.pi / 7, 0, 0]))
    # gt_T_rc = SE3(translation=[1.05, 1.1, 1.2], rotation=SO3.exp([np.pi / 3, 0.3, 0.2]))
    #
    # T_rgs = []
    # T_cts = []
    # for i in range(10):
    #     T_rg = SE3(
    #         translation=np.random.uniform(-1, 1, size=3),
    #         rotation=SO3.exp(np.random.uniform(-np.pi, np.pi, size=3)),
    #     )
    #     T_ct = gt_T_rc.inverse() * T_rg * gt_T_gt
    #     T_rgs.append(T_rg)
    #     T_cts.append(T_ct)

    # T_rgs, T_cts = prepare_data()
    # T_gt, T_rc = hand_eye(T_rgs, T_cts)

    # TODO
    detect_and_save_example()
    # detect_and_save()
    cam, dkt = load_arrays_from_file()
    T_cam, T_dkt = prepare_data(cam, dkt)
    T_gt, T_rc= hand_eye(T_cam, T_dkt)

    print(T_rc)
    # print(gt_T_rc)
    # print('---')
    print(T_gt)
    # print(gt_T_gt)



