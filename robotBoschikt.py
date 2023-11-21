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

#  based on BlueBot and Bosch Toolbox
#  by  O. Certik, V. Smutny, P. Krsek, M. Matousek


def robotBoschikt(robot, pos):
    """
    Inverse kinematic task for robot Bosch.
    :param robot: Robot Bosch instance.
    :param pos: Coordinates of robot position in world coordinates.
    :return: Coordinates of robot position in joint coordinates (degrees).
    """

    pos = np.array(pos).astype(float)
    c = np.sqrt(pos[0] ** 2 + pos[1] ** 2)
    v = (c ** 2 - robot.L1 ** 2 - robot.L2 ** 2) / (2 * robot.L1 * robot.L2)
    if abs(v) > 1.0:
        return []

    b = np.arccos(v) * 180.0 / np.pi
    g = np.arccos((c ** 2 + robot.L1 ** 2 - robot.L2 ** 2) / (2 * robot.L1 * c)) * 180.0 / np.pi

    if pos[0] == 0:
        d = 90.0 * np.sign(pos[1])
    else:
        d = np.arctan(pos[1] / pos[0]) * np.sign(pos[0]) * 180.0 / np.pi

    deg = np.empty((2,4))
    deg[0, 3] = pos[3]
    deg[0, 2] = -pos[2]
    deg[0, 1] = b
    deg[0, 0] = 90.0 - d - g
    deg[1, 3] = pos[3]
    deg[1, 2] = -pos[2]
    deg[1, 1] = -b
    deg[1, 0] = 90.0 - d + g

    if pos[0] != 0:
        deg[:, :2] = deg[:, :2] * np.sign(pos[0])

    if pos[0] < 0:
        p = deg[0, :]
        deg[0, :] = deg[1, :]
        deg[1, :] = p

    return deg