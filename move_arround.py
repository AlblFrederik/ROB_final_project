import argparse
import time

import numpy as np

from CRS_commander import Commander
# from demo.im_proc import *
from graph import Graph
from interpolation import *
from robCRSgripper import robCRSgripper
from robotCRS import robCRS93, robCRS97
from robotics_toolbox.core import SE3, SO3
from vision import Camera, Detector


def run_fors():
    dkt_list = []
    aruco_list = []

    tty_dev = r"/dev/ttyUSB0"
    skip_setup = False
    max_speed = None
    reg_type = None
    action = 'home'
    spline = 'poly'
    order = 2
    robot = robCRS93()
    hhirc = [-181650, -349, -62200, 99200, 8300, -96500]
    cords = np.array([0, -45, -45, 0.0, 0, 0])

    commander = Commander(robot)  # initialize commander
    commander.open_comm(tty_dev, speed=19200)  # connect to control unit
    camera = Camera()
    commander.init(reg_type=reg_type, max_speed=max_speed, hard_home=True)
    print("here")
    for i in range(5):
        # cords = np.array([spodni, sklon1, sklon2, hand vrtacka, gripr sklon, griper vrtacka])
        cords = np.array([-2 * i, -20 + i, -60 + 5 * i, 0.0, 0, 0])
        cord_IRC = commander.anglestoirc(cords)
        commander.coordmv(cord_IRC, relative=False)
        img = camera.get_image()
        Detector.get_all(img)
        time.sleep(2)

    for i in range(7):
        # cords = np.array([spodni, sklon1, sklon2, hand vrtacka, gripr sklon, griper vrtacka])
        cords = np.array([10 - 4 * i, -20, -90 - 2 * i, 0.0, 5 + 3 * i, 0])
        cord_IRC = commander.anglestoirc(cords)
        commander.coordmv(cord_IRC, relative=False)
        time.sleep(2)

    cords = np.array([0, -45, -45, 0.0, 0, 0])
    dktres = robot.dkt(robot, cords)
    print(dktres)

    rot = SO3.exp(dktres[3:])
    se3_matrix = SE3(dktres[:3], rot)


if __name__ == "__main__":
    img = Camera.get_image()
    data = Detector.get_all(img, visualize=True)
    print(data)