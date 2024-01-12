import argparse
import os
import time

import cv2
import numpy as np

from CRS_commander import Commander
# from demo.im_proc import *
from graph import Graph
from interpolation import *
from robCRSgripper import robCRSgripper
from robotCRS import robCRS93
from robotics_toolbox.core import SE3, SO3
from vision import Camera, Detector
from robCRSdkt import robCRSdkt
from robCRSgripper import robCRSgripper
from robCRSikt import robCRSikt
import data as numbers
from runner_utils import Robot


def move_loop(initial_cords, increment_cords, robot, camera, detector, n=10):
    initial_cords = np.array(initial_cords)
    increment_cords = np.array(increment_cords)
    camera_cords_list = []
    dkt_cord_list = []
    for i in range(n):
        cords = np.array(initial_cords + i * increment_cords)
        robot.move_to_q_position(cords)
        time.sleep(2)
        dkt_cords = robot.get_xyz_dkt(cords)
        path = f"data/{str(dkt_cords)}z.png"
        print(path)
        camera.save_img(path)
        # bricks, ids = detector.get_all(img)
        # camera_cords_list.append(bricks)
        # dkt_cords = robot.get_xyz_dkt(cords)
        # dkt_cord_list.append(dkt_cords)
    return camera_cords_list, dkt_cord_list


def run_motion():
    tty_dev = r"/dev/ttyUSB0"
    detector = Detector(False)
    camera = Camera(False)
    robot = Robot(tty_dev, False)

    camera_cords_list = []
    dkt_cord_list = []
    # init_cords = [[-2, -20, -60, 0.0, 0, 0], [-2, -20, -60, 0.0, 5, 0], [0, -45, -45, 0.0, 0, 0], [0, -45, -45, 5, 0, -5], [2, -30, -25, 1, 0, 3]]
    # incr_cords = [[1, 0.2, 0, 1, 0.6, 0.3], [0, 1, 0.5, -2, 0, 0], [1, 0.3, -1, -1, -2, 0], [2, 0, 1.5, 1, 0, -1], [2, -1, 1, 0, 0, 0]]
    init_cords = [[-2, -20, -60, 0.0, 0, 0], [-2, -20, -60, 0.0, 5, 0]]
    incr_cords = [[1, 0.2, 0, 1, 0.6, 0.3], [0, 1, 0.5, -2, 0, 0]]
    for i in range(len(init_cords)):
        cam, dkt = move_loop(init_cords[i], incr_cords[i], robot, camera, detector, 2)
        camera_cords_list.append(cam)
        dkt_cord_list.append(dkt)
    # camera_cords_list = np.array(camera_cords_list).flatten()
    # dkt_cord_list = np.array(dkt_cord_list).flatten()
    camera_cords_list = np.array(camera_cords_list).squeeze()
    dkt_cord_list = np.array(dkt_cord_list).squeeze()
    return camera_cords_list, dkt_cord_list

def get_calib_data():
    OK = 0
    ERROR = 0
    tty_dev = r"/dev/ttyUSB0"
    detector = Detector(False)
    camera = Camera(False)
    robot = Robot(tty_dev, True)
    camera_cords_list = []
    dkt_cord_list = []
    X = np.linspace(525, 700, 5)
    Y = np.linspace(-200, 50, 5)
    Z = np.linspace(100, 700, 12)
    #angle_z = np.linspace(-)
    for z in Z:
        for x in X:
            for y in Y:
                try:
                    xyz = [x, y, z, 0, 0, 0]
                    robot.move_xyz(xyz)
                    path = f"data/{str([x, y, z, 0, 0, 0])}z.png"
                    print(path)
                    camera.save_img(path)
                    OK += 1
                except:
                    ERROR +=1
            print(f"SUM: {OK+ERROR}, OK{OK}, ERROR{ERROR}")


    # x = +700  +350
    # y = -200  +60
    # z = +700 +100


def tbm(vector):
    #[0.54878762 - 0.10620403  1.22170883]
    #[-0.37403561  0.3633731 - 0.01518165]
    t = vector[:3]
    r_vec = np.array(vector[3:])
    t = np.array(t) * 1000
    r =  SO3.exp(r_vec)
    transform = SE3(t, r)
    T = SE3([548.78762, -106.20403,  1221.70883], SO3.exp([np.pi,  0, 0]))
    trans = T * transform
    t_vec = trans.translation
    r_vec = trans.rotation.log()
    return np.array([t_vec, r_vec]).flatten()

def tbc(cam_data, dkt_data):
    cam_t = cam_data[:3]
    cam_r = cam_data[3:] #np.flip(cam_data[3:], axis = 0)
    dkt_t = np.array(dkt_data[:3]) / 1000
    angle_x, angle_y, angle_z = np.array(dkt_data[3:])/180 * np.pi
    dkt_r = SO3.rx(angle_x) * SO3.ry(angle_y) * SO3.rz(angle_z)
    T_cam = SE3(cam_t, SO3.exp(cam_r))
    T_dkt = SE3(dkt_t, dkt_r)
    return T_dkt * T_cam.inverse()

def load_data():
    detector = Detector(False)
    img_list = os.listdir("data")
    transforms = []
    OK = 0
    for img in img_list:
        image = cv2.imread(f"./data/{img}")
        str_name = img.split("z")[0].replace('[', '').replace(']', '')
        #print(img)

        dkt_data = np.array(np.fromstring(str_name, dtype=float, sep=','))
        #print(dkt_data)
        cam_data, ids = detector.get_all(image)
        if cam_data is not None and len(cam_data) > 0:
            transform = tbc(cam_data[0], dkt_data)
            transforms.append(transform)
            OK += 1
    print(f"OK {OK/len(img_list)}")
    t_vec = [0, 0, 0]
    r_vec = [0, 0, 0]
    for trans in transforms:
        t_vec += trans.translation
        r_vec += trans.rotation.log()
    t_vec = np.array(t_vec) / len(transforms)
    r_vec = np.array(r_vec) / len(transforms)
    print(t_vec)
    print(r_vec)

    return transforms


def get_brick():
    # [0, -65, -70, 0.0, 30, 0] above brick
    # [0, -79, -72, 0.0, 53, 0] brick
    tty_dev = r"/dev/ttyUSB0"
    robot = Robot(tty_dev, False)
    robot.move_to_q_position([0, -65, -70, 0.0, 30, 0])
    robot.open_gripper()
    robot.move_to_q_position([0, -79, -72, 0.0, 53, 0])
    robot.close_gripper()
    robot.move_to_q_position([0, -65, -70, 0.0, 30, 0])


def boundaries_test():
    tty_dev = r"/dev/ttyUSB0"
    robot = Robot(tty_dev, False)
    detector = Detector(False)
    camera = Camera(True)
    print("blabla")
    xyz = [600, -0, 100, - 3, 0, 0]
    robot.move_xyz(xyz)
    img = camera.get_image()
    #x = +700  +350
    #y = -200  +60
    #z = +700 +100

def test_fcn():
    tty_dev = r"/dev/ttyUSB0"
    robot = Robot(tty_dev, False)
    detector = Detector(True)
    camera = Camera(False)
    robot.move_xyz([610, 0, 600, 0, 0, 0])
    img = camera.get_image()
    brick, id = detector.get_all(img)
    res = tbm(brick[0])
    print(f"Brick from camera in xyz {brick[0]}")
    print(f"Brick in robot cords {res}")
    t_vec = np.array(res[:3])
    r_vec = np.array(res[3:]) * (180 / np.pi)
    res = np.array([t_vec, r_vec]).flatten()
    kloubove_cords = robot.get_q_ikt(res)
    res = res * (np.pi / 180)
    print(res)
    print(robot.get_xyz_dkt(kloubove_cords))

    # robot.move_to_q_position(res[0])
    kostka = [-0.21908474,  0.06414142,  1.22211151,  2.24148191, - 2.19595512,  0.07352479]
    print(kostka)


def homography_calibration():
    get_homography_data()
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

def get_homography_data():
    OK = 0
    ERROR = 0
    tty_dev = r"/dev/ttyUSB0"
    detector = Detector(False)
    camera = Camera(False)
    robot = Robot(tty_dev, False)
    X = np.linspace(525, 700, 5)
    Y = np.linspace(-200, 50, 5)
    z = 100
    for x in X:
        for y in Y:
            try:
                xyz = [x, y, z, 0, 0, 0]
                robot.move_xyz(xyz)
                path = f"data_homography/{str([x, y, z, 0, 0, 0])}z.png"
                print(path)
                camera.save_img(path)
                OK += 1
            except:
                ERROR += 1
    print(f"SUM: {OK + ERROR}, OK{OK}, ERROR{ERROR}")


def pick_up_brick():
    tty_dev = r"/dev/ttyUSB0"
    rob = Robot(tty_dev, True)
    rob.move_to_q_position([0, -65, -70, 0.0, 30, 0])
    rob.open_gripper()
    rob.move_to_q_position([0, -79, -72, 0.0, 53, 0])
    rob.close_gripper()
    rob.move_to_q_position([0, -65, -70, 0.0, 30, 0])


if __name__ == "__main__":
    # detect_and_save()
    pick_up_brick()
    homography = homography_calibration()

    # get_homography_data()
    # load_data()
    # boundaries_test()
    # get_brick()
    # camera_list, dkt_list = run_motion()
    # load_data()
    # cords = np.array([spodni, sklon1, sklon2, hand vrtacka, gripr sklon, griper vrtacka])
    #[0.54878762 - 0.10620403  1.22170883]
    #[-0.37403561  0.3633731 - 0.01518165]

