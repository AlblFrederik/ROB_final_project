# Coordinated Spline Motion and Robot Control Project
# 
# Copyright (c) 2017 Olga Petrova <olga.petrova@cvut.cz>
# Advisor: Pavel Pisa <pisa@cmp.felk.cvut.cz>
# FEE CTU Prague, Czech Republic
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# In 2017, project funded by PiKRON s.r.o. http://www.pikron.com/

''' Module provides functions for robot object initialization '''

import numpy as np

from robCRSdkt import robCRSdkt
from robCRSgripper import robCRSgripper, robCRSgripperinit
from robCRSikt import robCRSikt


#  based on BlueBot and Bosch Toolbox
#  by  O. Certik, V. Smutny, P. Krsek, M. Matousek

class robCRS:
    """Common specifications for CRS robots"""
    def __init__(self):

        self.portname = 'COM4'
        self.controller = 'MARS8'
        self.demoname = 'robCRSdemo'

        self.DOF = 6

        # Robot link dimensions (these constants are read from documentation)
        inch2mm = 25.4  # conversion constant
        self.L1 = 13 * inch2mm - 25  # J2 above working desk
        self.L2 = 12 * inch2mm  # J2-J3
        self.L3 = 13 * inch2mm  # J3-J5
        self.L4 = 3 * inch2mm  # J5-flange
        self.L5 = inch2mm * (3.78 + 0.5)  # gripper length
        self.L6 = inch2mm * 1  # gripper finger length

        # Posible change of coordinate direction
        self.direction = np.array([1, 1, -1, -1, -1, -1])

        # IRC per rotation of motor.
        self.irc = np.array([1000, 1000, 1000, 500, 500, 500])  # from manual, it should work
        # motors' gearing
        self.gearing = np.array([100, 100, 100, 101, 100, 101])  # from manual
        # degtoirc=1 degree in IRC
        self.degtoirc = self.irc * self.direction * self.gearing * 4 / 360

        self.activemotors = 'ABCDEF'
        # axes in hh_axes_list are in order of initialization
        self.hh_axes_list = 'BACDEF'
        self.control_axes_list = 'ABCDEF'
        self.coord_axes = 'ABCDEF'

        # DH notation
        self.offset = np.array([0.0, 270.0, 90.0, 0.0, 0.0, 0.0]) / 180 * np.pi
        self.sign = np.array([1, 1, 1, 1, 1, 1])
        self.d = [self.L1, 0, 0, self.L3, 0, self.L4 + self.L5 + self.L6]
        self.a = [0, self.L2, 0, 0, 0, 0]
        self.alpha = np.array([90.0, 0.0, 270.0, 90.0, 270.0, 0]) / 180 * np.pi
        self.base = np.eye(4)
        self.tool = np.array([[0.0,0.0,-1.0,0.0],
                              [0.0,1.0, 0.0,0.0],
                              [1.0, 0.0,0.0,0.0],
                              [0.0,0.0,0.0,1.0]])

        # Base position in degrees
        # (corresponds to position robot.hhirc in IRC)
        self.hhdeg = [0, 0, 0, 0, 0, 0]

        # Softhome position in degrees
        #  - Robot is moved to this position during startup and closing.
        #  - The position is set by calling bbsofthome
        self.shdeg = [0, 0, -45, 0, -45, 0]

        self.REGME = [32000, 32000, 32000, 32000, 32000, 32000]

        # Constants of PID regulators
        self.REGP = [10, 12, 70, 35, 45, 100]
        self.REGI = [80, 63, 50, 80, 65, 300]
        self.REGD = [300, 200, 200, 130, 230, 350]

        # min and max speeds (IRC/256/msec)
        self.minspeed = np.rint(self.defaultspeed / 5)
        self.maxspeed = np.rint(self.defaultspeed * 2)

        # min a max acceleration
        self.minacceleration = np.rint(self.defaultacceleration / 5)
        self.maxacceleration = np.rint(self.defaultacceleration * 2)

        # smutneho hodnoty robot.REGCFG = [1370 1369 1369 1361 1369 1369]
        # D ma HW problem, protoze indexova znacka se casto nachazi o
        # (mnoho)otacek jinde
        # E nema indexy nikdy,
        # problem D a E identifikovan: V rozporu s dokumentaci ma HEDS na D a E
        # signal znacky v podmnozine kombinace kanalu AB=00, ackoliv by mel jit az
        # do hrany A mezi 0 a 1 (ci naopak), pricemz B ma byt nula. Cip, ktery
        # zpracovava signaly to jinak neumi.
        # Potencialne to lze resit nejakym RC clankem mezi HEDS a budicem v robotu,
        # daji se ocekavat problemy s ruznymi rychlostmi. Od tohoto reseni je
        # upusteno.
        # V teto verzi tedy D a E najizdi jen na MARK, nikoliv na indexovou znacku.
        # robot.REGCFG = [1494 1494 1494 1484 1476 1492]# D melo 1498,
        # robot.REGCFG = [1492 1494 1494 1484 1476 1492]# D melo 1498,

        # Uprava rychlosti HOME
        # robot.REGCFG = [1490 1491 1491 1482 1474 1490]# D melo 1498,

        # Speed reduced due to power suply problem
        self.REGCFG = [1489, 1490, 1490, 1481, 1474, 1490]  # D melo 1498,

        # CRS robot shall not release after some time
        self.IDLEREL = 1200

        # homing sequence TODO
        # robot.hardhome = robCRShardhome


        self.timeout = 200

        # Gripper parameters

        self.gripper = robCRSgripper
        self.gripper_init = robCRSgripperinit


        # Kind of gripper (CRSGripper, Magnetic)
        self.gripper_type = 'CRSGripper'

        # Analog input of position sensor
        self.gripper_ADC = 10
        # Maximal curent limit (0-255)
        # Value 16 corresponds to 250mA when feedback is 500
        self.gripper_current = 16
        # Limitation constant (feeadback from overcurrent)
        self.gripper_feedback = 500
        # Maximal energy limits voltage on motor
        # (0 - 32000 corresponds to 0-24V)
        # 2010-02-17 Martin Matousek note:
        # Manual says that gripper can survive up to 15 Volts, but
        # force more than 75# must not be used longer then 15 second. Thus
        # safe power seems to be 11.25 V. We are using more conservative value here
        # 10/32 * 24 = 7.5 V

        self.gripper_REGME = 10000
        # Maximal speed
        self.gripper_REGMS = 500
        # Axis configuration word
        self.gripper_REGCFG = 256
        # PID parameter of controller
        self.gripper_REGP = 200
        self.gripper_REGI = 0
        self.gripper_REGD = 100

        self.gripper_poll_time = 0.2
        self.gripper_poll_diff = 50

        self.ikt = robCRSikt
        self.dkt = robCRSdkt

        self.degtoirc = np.multiply(self.direction, np.multiply(self.irc, self.gearing)) * 4.0 / 360.0

class robCRS97(robCRS, object):
    """Robot specification: CRS97"""

    def __init__(self):
        self.description = 'CRS 1997 with gripper'

        # Base position in IRC - calibrated !!
        # (corresponds to position self.hhdeg in degrees)
        self.hhirc = [-182500, 252, -63625, 99200, 14300, -98150]

        # self bounds in IRC
        self.bound = [[-370000, -100000, -190000, -5000, -44000, -199500],
                      [10000, 100000, 65000, 202000, 72000, 4000]]

        # Speed reduced due to power suply problem
        self.defaultspeed = np.array([30, 8, 20, 30, 30, 55]) * 256
        self.defaultacceleration = np.rint(
            np.array([(30.0 / 400.0), (8.0 / 400.0), (20.0 / 400.0), (30.0 / 400.0), 1.0 / 2.0, 3.0 / 5.0]) * 256.0)

        # Gripper range
        self.gripper_bounds = [1001, 48]
        self.gripper_bounds_force = [1001 + 100, 48 - 1000]  # TODO must be verified

        # Axis(motor) of gripper
        self.gripper_ax = 'H'
        self.verbose = True
        # ------------------------------------------------------------------------------
        # Common parameters for CRS selfs
        super(robCRS97, self).__init__()

class robCRS93(robCRS, object):
    """Robot specification: CRS93"""

    def __init__(self):
        self.description = 'CRS 1993 with gripper'

        # Base position in IRC - calibrated !!
        # (corresponds to position self.hhdeg in degrees)
        self.hhirc = [-181650, -349, -62200, 99200, 8300, -96500]

        # self bounds in IRC
        self.bound = [[-370000, -100000, -190000,  -5000, -50000, -199500],
                 [10000,  100000,   63000, 203500,  67000,  4600]]

        # Speed reduced due to power suply problem
        self.defaultspeed = np.array([30, 8, 20, 30, 30, 75]) * 256
        self.defaultacceleration = np.rint(
            np.array([0.2, (8.0 / 200.0), (20.0 / 250.0), (30.0 / 200.0), 0.5, 3.0]) * 256.0)

        # Gripper range
        self.gripper_bounds = [840, 103]
        self.gripper_bounds_force = [840+1000, 103-1000]  # TODO must be verified

        # Axis(motor) of gripper
        self.gripper_ax = 'G'
        self.verbose = True
        # ------------------------------------------------------------------------------
        # Common parameters for CRS selfs
        super(robCRS93, self).__init__()
