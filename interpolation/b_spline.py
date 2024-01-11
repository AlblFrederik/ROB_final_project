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

''' Module provides functions for points interpolation using b-splines of 2nd and 3rd order '''

import numpy as np
from utils import param_correction

def _b_spline_2(x):
    """
    2nd order B-spline interpolation.
    :param x: Points to interpolate. Minimal number of points for spline of 2nd order is 5.
    :return: Parameters of spline.
    """

    n = len(x)
    if n < 5:
        raise ValueError("Not enough points for interpolation of 2nd order. "
                         "Minimal number for spline of 2nd order is 5.")

    M0 = np.array([[2, -4, 2], [0, 4, -3], [0, 0, 1]])
    M1 = np.array([[1, -2, 1], [1, 2, -2], [0, 0, 1]])
    M2 = np.array([[1, -2, 1], [1, 2, -3], [0, 0, 2]])

    lst = [0.5*x.T[:,0:3].dot(M0)]

    for i in range(1,n-3):
        lst.append(0.5*x.T[:,i:i+3].dot(M1))

    lst.append(0.5 * x.T[:,-3:].dot(M2))

    return lst


def _b_spline_3(x):
    """
    3rd order B-spline interpolation.
    :param x: Points to interpolate. Minimal number of points for spline of 3rd order is 7.
    :return: Parameters of spline.
    """

    n = len(x)
    if n < 7:
        raise ValueError("Not enough points for interpolation. "
                         "Minimal number for spline of 3rd order is 7.")

    M0 = np.array([[12, -36, 36, -12],
                   [0, 36, -54, 21],
                   [0, 0, 18, -11],
                   [0, 0, 0, 2]])

    M1 = np.array([[3, -9, 9, -3],
                   [7, 3, -15, 7],
                   [2, 6, 6, -6],
                   [0, 0, 0, 2]])

    M2 = np.array([[2, -6, 6, -2],
                   [8, 0, -12, 6],
                   [2, 6, 6, -6],
                   [0, 0, 0, 2]])

    M3 = np.array([[2, -6, 6, -2],
                   [8, 0, -12, 6],
                   [2, 6, 6, -7],
                   [0, 0, 0, 3]])

    M4 = np.array([[2, -6, 6, -2],
                   [7, -3, -15, 11],
                   [3, 9, 9, -21],
                   [0, 0, 0, 12]])

    lst = [1.0 / 12.0 * x.T[:, 0:4].dot(M0)]
    lst.append(1.0 / 12.0 * x.T[:, 1:5].dot(M1))

    for i in range(2, n - 5):
        lst.append(1.0/12.0 * x.T[:, i:i + 4].dot(M2))

    lst.append(1.0/12.0 * x.T[:, -5:-1].dot(M3))
    lst.append(1.0/12.0 * x.T[:, -4:].dot(M4))

    return lst


def interpolate(points, order):
    """
    Interpolation of points using B-spline.
    :param points: Points to interpolate.
    :param order: Order of B-spline.
    :return: Parameters of spline.
    """

    if order != 2 and order != 3:
        raise ValueError("Function supports only 2nd and 3rd order polynomials.")

    if order == 2:
        lst = _b_spline_2(points)
    if order == 3:
        lst = _b_spline_3(points)

    param = [[(lst[i][:])[:, 1:]] for i in range(len(lst))]
    param = np.reshape(param, [len(lst), points.shape[1] * order])

    param = param_correction(points[0], param, order)

    return param