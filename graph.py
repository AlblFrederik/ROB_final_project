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

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from matplotlib.widgets import Slider, Button, RadioButtons

from interpolation import *

''' Module provides graphing functions for spline trajectory visualisation '''


class Graph:

    """
    Class for graphing interface for spline trajectory. User can adjust interpolation algorithm and parameters.
    """

    def __init__(self, path):
        self.sol = path
        self.spline = 'poly'
        self.init_offset = self.sol[0]
        self.order = 2
        self.n_polynom, self.n_joints = self.sol.shape
        self.n_seg = int(len(self.sol)/3)
        self.lambda_ = 0.1

    def update(self, ax, fig, init=False):
        """
        Helper function for graph update after parameter change.
        :param ax: Axis to update.
        :param fig: Figure to update.
        :param init: Whether to initialize plots or use existing.
        """

        res = 100  # discretisation of x axis
        if self.spline == 'poly':
            params = poly.interpolate(self.sol)
            self.order = 3
            ax.set_title('3rd order polynomial')
        if self.spline == 'b-spline':
            params = b_spline.interpolate(self.sol, order=self.order)
            ax.set_title('%s order B-spline' % (str(self.order) + ('nd' if self.order == 2 else 'rd')))
        if self.spline == 'p-spline':
            num_segments = self.n_seg
            poly_deg = self.order
            penalty_order = 2
            lambda_ = self.lambda_
            params = p_spline.interpolate(self.sol, num_segments, poly_deg, penalty_order, lambda_)
            ax.set_title('%s order P-spline' % (str(self.order) + ('nd' if self.order == 2 else 'rd')))

        if self.order == 2:
            t = np.vstack((np.linspace(0, 1, res), np.linspace(0, 1, res) ** 2))
        if self.order == 3:
            t = np.vstack((np.linspace(0, 1, res), np.linspace(0, 1, res) ** 2, np.linspace(0, 1, res) ** 3))

        # shrinking of x axis is compensated
        t_long = np.linspace(0, self.n_polynom - 1, params.shape[0] * res)

        y = np.empty((self.n_joints, 0))
        off = self.init_offset[np.newaxis].T
        for i in range(params.shape[0]):
            yi = params[i].reshape((self.n_joints, self.order)).dot(t) + off
            off = yi[:, -1][np.newaxis].T
            y = np.append(y, yi, axis=1)

        t = np.arange(0, self.sol.shape[0])

        if init:
            self.plot = []
            for k in range(self.n_joints):
                self.plot += plt.plot(t, self.sol.T[k],'k')
                self.plot += plt.plot(t_long, y[k])

        else:
            for k in range(self.n_joints):
                self.plot[2*k].set_data(t, self.sol.T[k])
                self.plot[2*k+1].set_data(t_long, y[k])

        fig.canvas.draw_idle()

    def show_gui(self):
        """
        Show GUI for trajectory visualisation and parameter adjustment.
        """

        fig, ax = plt.subplots()
        plt.subplots_adjust(left=0.3, bottom=0.25)
        ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%0.e'))

        self.update(ax, fig, init=True)
        # plt.axis([0, 1, -10, 10])

        axcolor = 'lightgoldenrodyellow'
        ax_lambda = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)  # type: Axes
        ax_n_seg = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)  # type: Axes

        s_lam = Slider(ax_lambda, 'lambda', 0.001, 1.0, valinit=0.1)
        s_seg = Slider(ax_n_seg, 'number of segments', 1, len(self.sol), valinit=int(len(self.sol)/3))

        def sel_l_seg(val):
            self.lambda_ = s_lam.val
            self.n_seg = int(s_seg.val)
            self.update(ax, fig)

        s_lam.on_changed(sel_l_seg)
        s_seg.on_changed(sel_l_seg)

        resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
        button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')


        def reset(event):
            s_lam.reset()
            s_seg.reset()
        button.on_clicked(reset)

        r_ord = plt.axes([0.025, 0.35, 0.15, 0.15], facecolor=axcolor)
        radio_ord = RadioButtons(r_ord, ('2nd order', '3rd order'), active=0)

        rsp = plt.axes([0.025, 0.55, 0.15, 0.15], facecolor=axcolor)
        radio_spline = RadioButtons(rsp, ('Polynomial', 'B-spline', 'P-spline'), active=0)

        def set_order(label):
            self.order = int(label[0])
            self.update(ax, fig)

        def set_spline(label):
            if label == 'Polynomial':
                self.spline = 'poly'
                ax_lambda.set_visible(False)
                ax_n_seg.set_visible(False)
                r_ord.set_visible(False)
            if label == 'B-spline':
                self.spline = 'b-spline'
                ax_lambda.set_visible(False)
                ax_n_seg.set_visible(False)
                r_ord.set_visible(True)
            if label == 'P-spline':
                self.spline = 'p-spline'
                ax_lambda.set_visible(True)
                ax_n_seg.set_visible(True)
                r_ord.set_visible(True)
            self.update(ax, fig)

        radio_ord.on_clicked(set_order)
        radio_spline.on_clicked(set_spline)

        ax_lambda.set_visible(False)
        ax_n_seg.set_visible(False)
        r_ord.set_visible(False)

        plt.show()