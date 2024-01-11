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


def select_brick(bricks):
    if type(bricks[0] is float):
        brick = bricks
    else:
        brick = bricks[0]
    return brick


class Robot:

    def __init__(self, tty_dev, do_init=False):
        self.robot = robCRS93()
        self.commander = Commander(self.robot)  # initialize commander
        self.commander.open_comm(tty_dev)  # connect to control unit
        self.do_init = do_init
        self.cords_q = None
        self.gripper = None

    def __enter__(self):
        if self.do_init:
            max_speed = None
            reg_type = None
            self.commander.init(reg_type=reg_type, max_speed=max_speed, hard_home=True)

    def pick_up_brick(self, cords_xyz):
        # z +- (above)
        above_cords = cords_xyz
        self.move_to_xyz_position(above_cords)
        self.open_gripper()
        self.move_to_xyz_position(cords_xyz)
        self.close_gripper()
        self.move_to_xyz_position(above_cords)  # lift brick

    def move_to_xyz_position(self, cords_xyz):
        cords = self.get_q_ikt(cords_xyz)
        self.move_to_q_position(cords)

    def go_to_box(self, cords_q=None):
        if cords_q is None:
            cords_q = [30, -50, -72, 0.0, 53, 0]  # default box position
        cord_IRC = self.commander.anglestoirc(cords_q)
        self.commander.coordmv(cord_IRC, relative=False)
        robCRSgripper(self.commander, 0)  # open gripper

    def move_to_q_position(self, cords_q):
        cord_IRC = self.commander.anglestoirc(cords_q)
        self.commander.coordmv(cord_IRC, relative=False)
        self.cords_q = cords_q

    def get_q_ikt(self, cords_xyz):
        # TODO
        ikt_res = self.commander.find_closest_ikt()
        return []  # q cords

    def get_xyz_dkt(self, cords_q):
        # TODO
        dkt_res = robCRSdkt(self.robot, cords_q)
        return dkt_res

    def open_gripper(self):
        robCRSgripper(self.commander, 0)
        self.gripper = 0

    def close_gripper(self):
        robCRSgripper(self.commander, 1)
        self.gripper = 1
