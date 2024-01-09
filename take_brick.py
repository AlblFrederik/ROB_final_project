
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
from robCRSdkt import robCRSdkt
from robCRSgripper import robCRSgripper
from robCRSikt import robCRSikt
import data as numbers

def take_brick():
    tty_dev = r"/dev/ttyUSB0"
    robot = robCRS93()

    commander = Commander(robot)  # initialize commander
    commander.open_comm(tty_dev)  # connect to control unit
    ikt_cords_list = correct_ikt(robot, [0, -65, -70, 0.0, 30, 0])
    # commander.init(reg_type=reg_type, max_speed=max_speed, hard_home=True)
    camera = Camera()

    img = camera.get_image()
    bricks = Detector.get_all(img) *1000

    trans, rot = camera_to_robot(bricks)

    print(trans)
    print(rot)

    cord_IRC = commander.anglestoirc(ikt_cords_list[2])
    commander.coordmv(cord_IRC, relative=False)


def camera_to_robot(camera_coords):
    Tbc = [519.35341017, 8.00788134, 1059.0666881]
    # log_rotation = [0.52564371, - 0.57318661,  0.528072]
    log_rotation = [0, 0, 0]
    T_bc = SE3(Tbc, SO3.exp(log_rotation))

    # T_ct = SE3(camera_coords[:3], SO3.exp(camera_coords[3:]))
    T_ct = SE3(camera_coords[:3], SO3(0))

    T_bt = T_bc * T_ct

    translation, rotation = T_bt.translation, T_bt.rotation
    return translation, rotation


def correct_ikt(robot, cords_xyz):
    ikt = robCRSikt(robot, cords_xyz)
    return ikt


if __name__ == "__main__":
    take_brick()