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

import numpy as np
from robotBoschikt import robotBoschikt

#  based on BlueBot and Bosch Toolbox
#  by  O. Certik, V. Smutny, P. Krsek, M. Matousek


class robotBosch:
    """Robot specification: Bosch"""

    def __init__(self):
        # Robot link dimensions (these constants are read from documentation)
        self.L1 = 250
        self.L2 = 200

        # Pavel Krsek measured 27th of September 2003
        # self.degtoirc = [947.4865, 611.9475, ? 395.9611]
        self.irc = [-1080, -1080, 1080, 1080]

        # Joint motors' gearing
        #  self.gearing = [80 50 1 (3*11)] # original numbers
        # self.gearing = [78.9572, 50.9956, 1 32.9967]
        self.gearing = [78.9572, 50.9956, 37, 32.9967]
        # TODO joint 3 must be precisely calibrated

        # 11200 (134428) = 300mm

        # ------------------------------------------------------------------------------
        # General parameters (required)
        self.description = 'Bosch SR450s'

        self.DOF = 4
        self.activemotors = 'ABCD'
        self.hh_axes_list = 'CABD'
        self.control_axes_list = 'ABCD'
        self.coord_axes = 'ABCD'

        # Softhome position in IRC and degrees/mm.
        self.hhirc = [0, 0, 0, 0]
        self.hhdeg = [0, 0, 0, 0]

        # 1 degree/mm in IRC units
        self.degtoirc = np.multiply(self.irc, self.gearing) * 4.0 / 360.0

        # Robot bounds in deg/mm converted to IRC
        self.bound = np.array([[ -80536,  -73434, -146520,   -7919],
                                [  80536,  73434,    2220,  427637]])
        # 2010-01-29, Martin Matousek
        # this are the realy hard bounds. Its no possible to
        # to reach them with full speed.
        # robot.bound should be chosen with some reserve

        truebound = np.array([[-86200, -79000, -147000, -9000],[86900, 75400, 2500, 428000]])

        if not (np.all(self.bound[0] > truebound[0]) and np.all(self.bound[1] < truebound[1])):
            raise ValueError("Internal error in robot parameters.")


        # Speeds (IRC/256/msec)
        self.minspeed = [1000,  1000, 1000, 1000]
        self.maxspeed = [10000, 10000, 5000, 5000]
        self.defaultspeed = [10000, 10000, 5000, 5000]

        # Accelerations
        self.minacceleration = [0, 0, 1, 1]
        self.maxacceleration = [100, 100, 100, 100]
        self.defaultacceleration = [30, 30, 20, 30]

        self.controller = 'MARS8'

        # ------------------------------------------------------------------------------
        # General parameters (optional)

        self.ikt = robotBoschikt

        self.portname = 'COM2'  # corrected from COM4  26.3.2013 by Smutny
        self.BaudRate = 9600  # Comport baud rate (default 19200 baud)

        self.REGPWRON = 1  # user must press arm power button

        self.REGPWRFLG = 5
        self.REGCFG = [1498, 1498, 1370, 1498]

        # Set energies and PID
        self.REGME = [30000, 16000, 16000, 16000]
        self.REGP = [50, 50, 50, 50]
        self.REGI = [5, 5, 5, 5]
        self.REGD = [50, 50, 50, 50]

        # Bosch robot shall not release after some time
        self.IDLEREL = 0

        # This variable use bbmovex, for help see this fucntion.
        self.pose = 0  # TODO
