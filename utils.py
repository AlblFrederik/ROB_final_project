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


def param_correction(start, params, order):
    """
    Parameter correction for coordinate movement. Function prevents accumulating
    of numeric error in parameters due to float rounding during coordinate movement.
    :param start: Starting position of movement.
    :param params: Parameters of coordinate movement.
    :param order: Order of spline.
    :return: Corrected parameters.
    """
    pos = np.round(start.copy())
    oldpos = np.round(start.copy())
    pos_not_rounded = np.round(start.copy())
    for i in range(len(params)):
        param = np.reshape(params[i], [order, len(start)], order='F')
        pos += np.sum(np.round(param), axis=0)
        pos_not_rounded += np.sum(param, axis=0)
        diff = pos - pos_not_rounded

        param[0,:] = np.round(param[0]) - np.round(diff)
        pos = oldpos + np.sum(param, axis=0)
        oldpos = pos.copy()
        params[i] = np.reshape(np.round(param.T), [len(start)*order], order='C')
    return params