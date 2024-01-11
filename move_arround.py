import argparse
import time

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


def run_fors():
    tty_dev = r"/dev/ttyUSB0"

    skip_setup = False
    max_speed = None
    reg_type = None
    action = 'home'
    spline = 'poly'
    order = 2
    robot = robCRS93()
    detector = Detector(True)


    commander = Commander(robot)  # initialize commander
    commander.open_comm(tty_dev)  # connect to control unit
    camera = Camera()
    commander.init(reg_type=reg_type, max_speed=max_speed, hard_home=True)
    prev_pos = robCRSdkt(robot, [0, -45, -45, 0.0, 0, 0])
    print("here")

    res = robCRSikt(robot, [1, 0.2, 1, 0, 0, 0])
    print(res)

    camera_cords_list = []
    dkt_cord_list = []
    for i in range(7):
        # cords = np.array([spodni, sklon1, sklon2, hand vrtacka, gripr sklon, griper vrtacka])
        cords = np.array([-2 * i, -20 + i, -60 + 5 * i, 0.0, 0, 0])
        cord_IRC = commander.anglestoirc(cords)
        commander.coordmv(cord_IRC, relative=False)
        time.sleep(2)
        img = camera.get_image()
        bricks, ids = detector.get_all(img)
        camera_cords_list.append(bricks)
        dkt_cords = robCRSdkt(robot, cords)
        dkt_cord_list.append(dkt_cords)


    for i in range(7):
        # cords = np.array([spodni, sklon1, sklon2, hand vrtacka, gripr sklon, griper vrtacka])
        cords = np.array([10 - 4 * i, -20, -90 - 2 * i, 0.0, 5 + 3 * i, 0])
        cord_IRC = commander.anglestoirc(cords)
        commander.coordmv(cord_IRC, relative=False)
        time.sleep(2)
        img = camera.get_image()
        bricks, ids = detector.get_all(img)
        camera_cords_list.append(bricks)
        dkt_cords = robCRSdkt(robot, cords)
        #rot = SO3.exp(dkt_cords[3:])
        #se3_matrix = SE3(dkt_cords[:3], rot)
        dkt_cord_list.append(dkt_cords)

    for i in range(7):
        cords = np.array([-10 + 4 * i, -20 + 0.5*i, -90 - 2 * i, 0.0, 5 , ])
        cord_IRC = commander.anglestoirc(cords)
        commander.coordmv(cord_IRC, relative=False)
        time.sleep(2)
        img = camera.get_image()
        bricks, ids = detector.get_all(img)
        camera_cords_list.append(bricks)
        dkt_cords = robCRSdkt(robot, cords)
        # rot = SO3.exp(dkt_cords[3:])
        # se3_matrix = SE3(dkt_cords[:3], rot)
        dkt_cord_list.append(dkt_cords)

    for i in range(7):
        cords = np.array([-10 + 4 * i, -20 + 0.5 * i, -90 - 2 * i, 0.0, 5 + 3 * i, 0])
        cord_IRC = commander.anglestoirc(cords)
        commander.coordmv(cord_IRC, relative=False)
        time.sleep(2)
        img = camera.get_image()
        bricks, ids = detector.get_all(img)
        camera_cords_list.append(bricks)
        dkt_cords = robCRSdkt(robot, cords)
        # rot = SO3.exp(dkt_cords[3:])
        # se3_matrix = SE3(dkt_cords[:3], rot)
        dkt_cord_list.append(dkt_cords)


    cords = np.array([0, -45, -45, 0.0, 0, 0])
    dktres = robot.dkt(robot, cords)
    print(dktres)

    return camera_cords_list, dkt_cord_list


def get_tbc(cam_list, dkt_list):
    sum_cam = np.zeros(6)
    sum_dkt = np.zeros(6)

    for i in range(len(cam_list)):
        sum_cam += cam_list[i]
        sum_dkt += dkt_list[i]
    sum_cam /= len(cam_list)
    sum_dkt /= len(dkt_list)
    rot_cam = SO3.exp(sum_cam[3:])
    sum_dkt[3:] = sum_dkt[3:] * np.pi / 180
    rot_dkt = SO3.rx(sum_dkt[3]) * SO3.ry(sum_dkt[4]) * SO3.rz(sum_dkt[5])
    # rot_dkt = SO3.exp(sum_dkt[3:])
    se3_dkt = SE3(translation=sum_dkt[:3], rotation=rot_dkt)
    se3_cam = SE3(translation=sum_cam[:3], rotation=rot_cam)
    Tbc = se3_dkt * se3_cam.inverse()
    return Tbc


if __name__ == "__main__":
    det = Detector(True)
    cam, dkt = run_fors()
    #cam_list = numbers.cam_list
    #cam_list[:,:3] *= 1000
    #dkt_list = numbers.dkt_list
    print(cam, dkt)
    T_bc = get_tbc(cam, dkt)

    print(T_bc)

    # img = Camera.get_image()
    # data = det.get_all(img, visualize=False)

    # Tcb = hand_eye(dkt_list, cam_list)
    # print(Tcb)
    #!!!Tcb = [519.35341017, 8.00788134,  1059.0666881], log_rotation=[ 0.52564371 -0.57318661  0.528072 ]

    # [0, -65, -70, 0.0, 30, 0] above brick
    # [0, -79, -72, 0.0, 53, 0] brick

    # dkt
    # [7.12875065e+02,   1.34158390e-13,   1.46094636e+02,   2.25383061e-14, 1.50000000e+01,   9.46124180e-15]

    # dkt home
    # [7.56038147e+02,   1.63999197e-13,   5.20726147e+02,   2.45584825e-14, - 1.59027734e-14, - 3.50835465e-15]