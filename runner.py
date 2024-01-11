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
from runner_utils import *


case = 0
LOCATION_COUNTER = 0

tty_dev = r"/dev/ttyUSB0"
camera = Camera(visualize=False)
robot = Robot(tty_dev=tty_dev, do_init=True)

robot.move_to_q_position([0, -45, -45, 0.0, 0, 0])
brick = [0, -45, -45, 0.0, 0, 0]

while True:

    if case == 0:
        img = camera.get_image()
        bricks = Detector.get_all(img)
        if bricks is None or len(bricks) == 0:
            LOCATION_COUNTER += 1
            if LOCATION_COUNTER > 0:
                exit()
            else:
                new_cords = robot.cords_q + [10, 0, 0, 0.0, 0, 0]
                robot.move_to_q_position(new_cords)
        else:
            LOCATION_COUNTER = 0
            brick = select_brick(bricks)
            case = 1

    if case == 1:
        ...

        robot.pick_up_brick(brick)

        # move above brick
        # pick up brick

    if case == 2:
        ...
        # have brick
        # move to box and drop