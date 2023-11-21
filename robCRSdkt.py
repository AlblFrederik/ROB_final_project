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


def robCRSdkt(robot, pos):
    """
    Direct kinematic task - robot CRS.
    :param robot: CRS robot instance.
    :param pos: Coordinates of robot position in joint coordinates (degrees).
    :return: Coordinates of robot position in world coordinates.
    """
    T = np.eye(4)
    pos = np.array(pos) / 180.0 * np.pi

    for i in range(6):
        tz = np.eye(4)
        tz[2, 3] = robot.d[i]
        rz = np.eye(4)
        o = -robot.offset[i]
        rz[:2, :2] = np.array([[np.cos(o + pos[i]), -np.sin(o + pos[i])],
                               [np.sin(o + pos[i]), np.cos(o + pos[i])]])
        tx = np.eye(4)
        tx[0, 3] = robot.a[i]
        rx = np.eye(4)
        a = robot.alpha[i]
        rx[1:3, 1:3] = np.array([[np.cos(a), -np.sin(a)],
                                 [np.sin(a), np.cos(a)]])
        T = T.dot(tz).dot(rz).dot(tx).dot(rx)
    T = T.dot(robot.tool)
    coord = T[:3, 3]
    a3 = np.arctan2(T[1, 0], T[0, 0]) / np.pi * 180
    a4 = np.arcsin(-T[2, 0]) / np.pi * 180
    a5 = np.arctan2(T[2, 1], T[2, 2]) / np.pi * 180
    coord = np.hstack((coord, a3, a4, a5))
    return coord
