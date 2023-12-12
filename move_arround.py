import argparse
import numpy as np

from CRS_commander import Commander
# from demo.im_proc import *
from graph import Graph
from interpolation import *
from robCRSgripper import robCRSgripper
from robotCRS import robCRS93, robCRS97
from robotics_toolbox.core import SE3, SO3

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

commander.init(reg_type=reg_type, max_speed=max_speed, hard_home=True)
cord_IRC = commander.anglestoirc(cords)
print(cord_IRC)
commander.coordmv(cord_IRC, min_time=10)
print("here")
dktres = robot.dkt(robot, cords)
print(dktres)

rot = SO3.exp(dktres[3:])
se3_matrix = SE3(dktres[:3], rot)
