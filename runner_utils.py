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
        self.commander = Commander(self.robot)
        self.commander.open_comm(tty_dev)
        self.do_init = do_init

    def __enter__(self):
        if self.do_init:
            max_speed = None
            reg_type = None
            self.commander.init(reg_type=reg_type, max_speed=max_speed, hard_home=True)


def pick_up_brick(cords_xyz):
    pass


def move_to_xyz_position(cords_xyz):
    pass


def go_to_box():
    pass