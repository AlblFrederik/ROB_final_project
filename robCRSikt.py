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

#  based on ROBCRSIKT by Pavel Krsek, Michal Havlena
#  for BlueBot and Bosch Toolbox

def robCRSikt(robot, pos):
    """
    Inverse kinematic task - robot CRS.
    :param robot: CRS robot instance.
    :param pos: Coordinates of robot position in world coordinates.
    :return: Coordinates of robot position in joint coordinates (degrees).
    """
    
    pos = np.array(pos).astype(float)
    pos[3:] = pos[3:] / 180.0 * np.pi

    myeps = 10000 * 2.2204e-16 # equality tolerance
    par1 = 0 # if infinite number of solutions, theta1=par1
    par4 = 0 # if infinite number of solutions, theta4=par4

    # T = base * A01 * A12 * A23 * A34 * A45 * A56 * A76 * tool
    A76 = np.eye(4)
    A76[2][3] = robot.d[5]
    T = np.array([[np.cos(pos[3]) * np.cos(pos[4]), - np.sin(pos[3]) * np.cos(pos[5]) + np.cos(pos[3]) * np.sin(pos[4]) * np.sin(pos[5]),
        np.sin(pos[3]) * np.sin(pos[5]) + np.cos(pos[3]) * np.sin(pos[4]) * np.cos(pos[5]), pos[0]],
        [np.sin(pos[3]) * np.cos(pos[4]), np.cos(pos[3]) * np.cos(pos[5]) + np.sin(pos[3]) * np.sin(pos[4]) * np.sin(pos[5]),
         - np.cos(pos[3]) * np.sin(pos[5]) + np.sin(pos[3]) * np.sin(pos[4]) * np.cos(pos[5]), pos[1]],
         [- np.sin(pos[4]), np.cos(pos[4]) * np.sin(pos[5]), np.cos(pos[4]) * np.cos(pos[5]), pos[2]],
         [0, 0, 0, 1]])
    W = np.linalg.inv(robot.base).dot(T.dot(np.linalg.inv(robot.tool).dot(np.linalg.inv(A76))))
    # X = A01 * A12 * A23 * [0 0 0 1]' because A34*A45*A57==R34*R45*R56 is pure rotation
    X = W.dot(np.array([0, 0, 0, 1])[np.newaxis].T).T[0]

    # solve joints 1, 2, 3
    J = []
    b = X[2] - robot.d[0]
    if abs(X[0]) < myeps and abs(X[1]) < myeps: # arm going straight up
        if abs(b - robot.d[3] - robot.a[1]) < myeps: # full length
            J.append([par1, 0, 0])
        elif b < robot.d[3] + robot.a[1]: # can reach
            J.append([ par1, - np.arccos((robot.a[1] ** 2 + b ** 2 - robot.d[3] ** 2) / (2 * robot.a[1] * b)),
            np.pi - np.arccos((robot.a[1] ** 2 + robot.d[3] ** 2 - b ** 2) / (2 * robot.a[1] * robot.d[3]))])
            J.append([par1, np.arccos((robot.a[1] ** 2 + b ** 2 - robot.d[3] ** 2) / (2 * robot.a[1] * b)), - np.pi + np.arccos(
            (robot.a[1] ** 2 + robot.d[3] ** 2 - b ** 2) / (2 * robot.a[1] * robot.d[3]))])
        else: # cannot reach
            J = [np.nan, np.nan, np.nan]

    else:
        c = np.sqrt(b ** 2 + X[0] ** 2 + X[1] ** 2)
        if abs(c - robot.d[3] - robot.a[1]) < myeps: # full length
            J.append([np.arctan2(X[1], X[0]) - np.pi / 2 + np.arcsin(b / c), 0])
            J.append([np.arctan2(-X[1], -X[0]), np.pi / 2 - np.arcsin(b / c), 0])
        elif c < robot.d[3] + robot.a[1]: # can reach
            theta2 = np.pi / 2 - np.arcsin(b / c) + np.arccos((robot.a[1] ** 2 + c ** 2 - robot.d[3] ** 2) / (2 * robot.a[1] * c))
        # can be bigger than np.pi!!! 
            if theta2 > np.pi:
                theta2 = theta2-2 * np.pi

            J.append(np.array([np.arctan2(X[1], X[0]), - theta2, np.pi - np.arccos((robot.a[1] ** 2 + robot.d[3] ** 2 - c ** 2) / (2 * robot.a[1] * robot.d[3]))]))
            J.append(np.array([np.arctan2(X[1], X[0]), - np.pi / 2 + np.arcsin(b / c) + np.arccos((robot.a[1] ** 2 + c ** 2 - robot.d[3] ** 2) / (2 * robot.a[1] * c)),
            - np.pi + np.arccos((robot.a[1] ** 2 + robot.d[3] ** 2 - c ** 2) / (2 * robot.a[1] * robot.d[3]))]))
            J.append(np.array([np.arctan2(-X[1], -X[0]), theta2, - np.pi + np.arccos((robot.a[1] ** 2 + robot.d[3] ** 2 - c ** 2) / (2 * robot.a[1] * robot.d[3]))]))
            J.append(np.array([np.arctan2(-X[1], -X[0]), np.pi / 2 - np.arcsin(b / c) - np.arccos((robot.a[1] ** 2 + c ** 2 - robot.d[3] ** 2) / (2 * robot.a[1] * c)),
            np.pi - np.arccos((robot.a[1] ** 2 + robot.d[3] ** 2 - c ** 2) / (2 * robot.a[1] * robot.d[3]))]))
        else: # cannot reach
            J = [np.nan, np.nan, np.nan]


    deg = []
    toolJ = np.eye(4)
    toolJ[2][3] = robot.d[3]
    for j in range(np.array(J).shape[0]):
        nnn = [np.isnan(a) for a in J]
        if not np.any(nnn):
            # direct kinematics for first 3 joints; inversed
            dif = (J[j] -robot.offset[:3])
            robot.theta = dif*robot.sign[:3]
            P = W
            for i in range(3):
                M = [[np.cos(robot.theta[i]), -np.sin(robot.theta[i]) * np.cos(robot.alpha[i]), np.sin(robot.theta[i]) * np.sin(robot.alpha[i]), robot.a[i] * np.cos(robot.theta[i])],
                [np.sin(robot.theta[i]), np.cos(robot.theta[i]) * np.cos(robot.alpha[i]), -np.cos(robot.theta[i]) * np.sin(robot.alpha[i]), robot.a[i] * np.sin(robot.theta[i])],
                [0, np.sin(robot.alpha[i]), np.cos(robot.alpha[i]), robot.d[i]],
                [0, 0, 0, 1]]
                P = np.linalg.inv(M).dot(P)
                # P = R34 * R45 * R56
            P = np.linalg.inv(toolJ).dot(P)

            # Euler Z - Y Z for joints 4, 5, 6
            if abs(P[2][2] - 1) < myeps: # np.cos(theta5) == 1
                deg.append(J[j].tolist() + [par4, 0, np.arctan2(P[1][0], P[0][0]) - par4])
            elif abs(P[2][2] + 1) < myeps: # np.cos(theta5) == -1
                deg.append(J[j].tolist() + [par4, np.pi, np.arctan2(P[1][0], -P[0][0]) + par4])
            else: # non - degenerate
                theta5 = np.arccos(P[2][2])
                deg.append(J[j].tolist() +[np.arctan2(P[1][2] * np.sign(np.sin(theta5)), P[0][2] * np.sign(np.sin(theta5))), - theta5,
                np.arctan2(P[2][1] * np.sign(np.sin(theta5)), -P[2][0] * np.sign(np.sin(theta5)))])
                deg.append(J[j].tolist() + [np.arctan2(P[1][2] * np.sign(np.sin(-theta5)), P[0][2] * np.sign(np.sin(-theta5))), theta5,
                np.arctan2(P[2][1] * np.sign(np.sin(-theta5)), -P[2][0] * np.sign(np.sin(-theta5)))])
        else:
            deg = J + [np.nan, np.nan, np.nan]

    deg = np.array(deg) * 180 / np.pi
    return deg