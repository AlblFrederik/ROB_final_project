#!/usr/bin/env python

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

''' Module provides functions for robot control via Mars 8 unit. '''

#  based on code by P. Pisa

import sys
import time

import numpy as np
import serial


class Commander:

    def __init__(self, robot, rcon=None):
        """
        Commander constructor.
        :param robot: Robot instance, e.g. robotBosch, robCRS97 or robCRS93.
        :param rcon: Serial interface.
        """
        self.robot = robot
        self.rcon = rcon #type: serial.Serial
        self.stamp = int(time.time() % 0x7fff)
        self.last_trgt_irc = None
        self.coordmv_commands_to_next_check = 0
        self.coord_axes = None

    def set_rcon(self, rcon):
        """
        Set communication interface.
        :param rcon: Serial interface.
        """
        if self.rcon is not None:
            self.rcon.close()
            self.rcon = None
        self.rcon = rcon

    def send_cmd(self, cmd):
        """
        Send command to command unit through serial interface.
        :param cmd: Command to send.
        """
        ba = bytearray(cmd, 'ascii')
        self.rcon.write(ba)

    def read_resp(self, maxbytes):
        """
        Read response from command unit.
        :param maxbytes: Max number of bytes to read.
        """
        resp = self.rcon.read(maxbytes)
        if resp is None:
            return None
        s = resp.decode("ascii")
        s = s.replace("\r\n", "\n")
        s = s.replace("\r", "\n")
        return s

    def irctoangles(self, a):
        """
        Convert IRC to degrees.
        :param a: IRC of angle.
        :return: Degrees corresponding to IRC.
        """
        j = np.atleast_2d(a).shape[1]
        n = np.atleast_2d(a).shape[0]
        if j != self.robot.DOF:
            raise ValueError("Wrong number of joints (%d, should be %d)." % (j, self.robot.DOF))

        b = np.divide((a - np.repeat(np.array([self.robot.hhirc]), n, axis=0)),
                        np.repeat(np.array([self.robot.degtoirc]), n, axis=0)) \
            + np.repeat(np.array([self.robot.hhdeg]), n, axis=0)
        if n == 1:
            return b[0]
        else:
            return b

    def anglestoirc(self, a):
        """
        Convert degrees to IRC.
        :param a: Degrees of angle.
        :return: IRC corresponding to degrees.
        """
        j = np.atleast_2d(a).shape[1]
        n = np.atleast_2d(a).shape[0]
        if j != self.robot.DOF:
            raise ValueError("Wrong number of joints (%d, should be %d)." % (j, self.robot.DOF))

        b = np.multiply((a - np.repeat(np.array([self.robot.hhdeg]), n, axis=0)),
                        np.repeat(np.array([self.robot.degtoirc]), n, axis=0)) \
            + np.repeat(np.array([self.robot.hhirc]), n, axis=0)
        if n == 1:
            return np.rint(b[0])
        else:
            return np.rint(b)

    def degtorad(self, d):
        """
        Convert degrees to radians.
        :param d: Angle in degrees.
        :return: Angle in radians.
        """
        return d * np.pi / 180.0

    def radtodeg(self, d):
        """
        Convert radians to degrees.
        :param d: Angle in radians.
        :return: Angle in degrees.
        """
        return d * 180.0 / np.pi

    def init_robot(self):
        """
        Initialize robot. Function performs all necessary settings of control unit.
        Initialization may require user to press ARM POWER button (placed on control unit).
        """
        self.sync_cmd_fifo()
        if hasattr(self.robot, 'REGPWRON') and self.robot.REGPWRON == 1:
            self.send_cmd('REGPWRON:%i\n'%self.robot.REGPWRON)
            print('Press ARM POWER button,\n')
            if sys.version_info[0] < 3:
                raw_input('press enter to continue...')
            else:
                input('press enter to continue...')
            self.send_cmd('REGPWRFLG:%i\n'%self.robot.REGPWRFLG)

        print('Resetting motors')
        # Purge
        self.send_cmd("PURGE:\n")
        self.send_cmd("STOP:\n")

        self.check_ready()
        self.wait_ready()

        self.set_speed_par(self.robot.defaultspeed)
        self.set_acc_par(self.robot.defaultacceleration)

        fields = ['REGME', 'REGCFG', 'REGP', 'REGI', 'REGD']

        for f in fields:
            param_list = getattr(self.robot, f, [])
            if param_list:
                for i in range(self.robot.DOF):
                    if self.robot.activemotors[i]:
                        self.send_cmd('%s%s:%i\n' % (f, self.robot.activemotors[i], param_list[i]))

        if hasattr(self.robot, 'IDLEREL'):
            self.send_cmd('IDLEREL:%i\n'%self.robot.IDLEREL)

        if hasattr(self.robot, 'gripper_init'):
            if self.robot.verbose:
                print('Gripper init.')
            self.robot.gripper_init(self)

        if self.robot.description[:3] == 'CRS':
            self.send_cmd('SPDTB:0,300\n')

    def sync_cmd_fifo(self):
        """
        Synchronize message queue.
        """
        self.stamp = (self.stamp + 1) & 0x7fff
        self.send_cmd('STAMP:%d\n' % self.stamp)
        buf='\n'
        while True:
            buf += self.read_resp(1024)
            i = buf.find('\n' + 'STAMP=')
            if i < 0:
                continue
            s = '%d'%self.stamp
            r = buf[i+7:]
            j = r.find('\n')
            if j < 0:
                continue
            r = r[0:j].strip()
            if r == s:
                break

    def check_ready(self, for_coordmv_queue = False):
        """
        Check robot is in "ready" state.
        :param for_coordmv_queue: Boolean, whether to check state for coordinate movement message queue.
        :return: Boolean, whether robot is ready or not.
        """
        a = int(self.query('ST'))
        s = ''
        if a & 0x8:
            s = 'error, '
        if a & 0x10000:
            s += 'arm power is off, '
        if a & 0x20000:
            s += 'motion stop, '
        if s:
            raise Exception('Check ready: %s.'%s[:-2])
        if for_coordmv_queue:
            return False if a & 0x80 else True
        else:
            return False if a & 0x10 else True

    def set_speed_par(self, params, force=False):
        """
        Set joint's speed parameters.
        :param params: Minimal and maximal speed for motors.
        :param force: Force set, ignores lower and upper bound of speed set in robot object.
        """
        for i in range(self.robot.DOF):
            if self.robot.activemotors[i] != '':
                if np.imag(params[i]) or params[i] == 0: # relative speed
                    r = np.imag(params[i])
                    if r < 0 or r > 1:
                        raise  Exception('Relative speed %i out of <0;1>'%i)
                    params[i] = round(self.robot.minspeed[i] * (1 - r) + self.robot.maxspeed[i] * r)
                    self.send_cmd('%s%s:%i\n'%('REGMS', self.robot.activemotors[i], params[i]))
                elif not force and (params[i] < self.robot.minspeed[i] or params[i] > self.robot.maxspeed[i]):
                    # speed is not inside lower and upper bound
                    raise Exception('Speed %d is out of bound'%i)
                else: # set the speed
                    self.send_cmd('%s%s:%i\n'%('REGMS', self.robot.activemotors[i], params[i]))

    def set_acc_par(self, params, force=False):
        """
        Set joint's acceleration parameters.
        :param params: Minimal and maximal acceleration for motors.
        :param force: Force set, ignores lower and upper bound of acceleration set in robot object.
        """
        for i in range(self.robot.DOF):
            if self.robot.activemotors[i] != '':
                if np.imag(params[i]) or params[i] == 0:  # relative acceleration
                    r = np.imag(params[i])
                    if r < 0 or r > 1:
                        raise  Exception('Relative acceleration %i out of <0;1>'%i)
                    params[i] = round(self.robot.minacceleration[i] * (1 - r) + self.robot.maxacceleration[i] * r)
                    self.send_cmd('%s%s:%i\n'%('REGACC', self.robot.activemotors[i], params[i]))
                elif not force and (params[i] < self.robot.minacceleration[i] or params[i] > self.robot.maxacceleration[i]):
                    # acceleration is not inside lower and upper bound
                    raise Exception('Acceleration %d is out of bound'%i)
                else: # set the speed
                    self.send_cmd('%s%s:%i\n'%('REGACC', self.robot.activemotors[i], params[i]))

    def init_communication(self):
        """
        Initialize communication through serial interface.
        """
        s = self.read_resp(1024)
        self.send_cmd("\nECHO:0\n")
        self.sync_cmd_fifo()
        s = self.query('VER')
        print('Firmware version : ' + s)

    def set_int_param_for_axes(self, param, val, axes_list=None):
        """
        Set parameter for a joint.
        :param param: Parameter to set.
        :param val: Value of parameter.
        :param axes_list: List of joints to set parameters to.
        """
        if axes_list is None:
            axes_list = self.robot.control_axes_list
        valstr = str(int(val))
        for a in axes_list:
            self.send_cmd(param + a + ':' + valstr + '\n')

    def set_max_speed(self, val, axes_list=None):
        """
        Set maximal speed of joints.
        :param val: Maximal speeds for joints in axes_list.
        :param axes_list: List of joints to set maximal speeds to.
        """
        self.set_int_param_for_axes(axes_list=axes_list, param='REGMS', val=val)

    def setup_coordmv(self, axes_list=None):
        """
        Setup coordinate movement of joints in axes_list.
        :param axes_list: List of joints.
        """
        if axes_list is None:
            axes_list = self.robot.coord_axes
        self.wait_ready()
        axes_coma_list = ','.join(axes_list)
        self.send_cmd('COORDGRP:' + axes_coma_list + '\n')
        self.wait_ready()
        self.coord_axes = axes_list
        self.last_trgt_irc = None

    def throttle_coordmv(self):
        """
        Throttle message queue for coordinate movement.
        :return: Boolean whether the queue is throttled.
        """
        throttled = False
        if self.coordmv_commands_to_next_check <= 0:
            self.coordmv_commands_to_next_check = 20
        else:
            self.coordmv_commands_to_next_check -= 1
            return
        while not self.check_ready(for_coordmv_queue = True):
            if not throttled:
                print('coordmv queue full - waiting')
            throttled = True
        return throttled

    def coordmv(self, pos, min_time=None, relative=False, disc=5):
        """
        Coordinate movement of joints.
        :param pos: Position to move to.
        :param min_time: Minimal time for the movement, if None movement is carried in minimal possible time.
        :param relative: Boolean, whether movement is relative to previous (current) position.
        :param disc: Discontinuity of movement, internal parameter, is to be found in control unit docs.
        """
        self.throttle_coordmv()
        self.send_cmd('COORDISCONT:%d'%disc + '\n')
        cmd = 'COORDMV' if not relative else 'COORDRELMVT'
        if (min_time is not None) and not relative:
            cmd += 'T'
        cmd += ':'
        if (min_time is not None) or relative:
            if min_time is None:
                min_time=0
            cmd += str(int(round(min_time)))
            if len(pos) > 0:
                cmd += ','
        pos = [int(round(p)) for p in pos]
        cmd += ','.join([str(p) for p in pos])
        self.send_cmd(cmd + '\n')
        if relative:
            if self.last_trgt_irc is None:
                raise ValueError("Relative movement is requested, but last_trgt_irc is None!")
            pos = [p + t for p, t in zip(pos, self.last_trgt_irc)]
            # pos = map(int.__add__, pos, self.last_trgt_irc)
        self.last_trgt_irc = pos

    def splinemv(self, param, order=1, min_time=None, disc=5):
        """
        Spline movement.
        :param param: Parameters of spline movement.
        :param order: Order of spline.
        :param min_time: Minimal time for the movement, if None movement is carried in minimal possible time.
        :param disc: Discontinuity of movement, internal parameter, is to be found in control unit docs.
        :return: Command for unit for specified parameters.
        """
        self.throttle_coordmv()
        self.send_cmd('COORDISCONT:%d' % disc + '\n')
        param = [int(round(p)) for p in param]
        cmd = 'COORDSPLINET'
        if min_time is None:
            min_time = 0
        cmd += ':'
        cmd += str(int(round(min_time)))
        cmd += ','
        cmd += str(int(round(order)))
        cmd += ','
        cmd += ','.join([str(p) for p in param])

        self.send_cmd(cmd + '\n')
        return cmd

    def axis_get_pos(self, axis_lst=None):
        """
        Get position of joints.
        :return: Position of active joints.
        """
        if axis_lst is None:
            axis_lst = self.robot.coord_axes
        resp = self.query('COORDAP')
        try:
            resp = np.fromiter(map(int, resp.split(',')), int)
        except:
            print('Error responce', resp)
        t = resp[0]
        pos = resp[-6:]
        return t, pos

    def hard_home(self, axes_list=None):
        """
        Robot hard homing. Returns joints in axes_list to home position.
        :param axes_list: List of joints to home.
        """
        # Hard-home
        if axes_list is None:
            axes_list = self.robot.hh_axes_list

        self.set_speed_par(self.robot.defaultspeed)
        self.set_acc_par(self.robot.defaultacceleration)

        self.last_trgt_irc = None

        for a in axes_list:
            self.send_cmd('HH' + a + ':\n')
            self.wait_ready()

    def soft_home(self, axes_list=None):
        """
        Robot soft homing. Returns joints in axes_list to home position.
        :param axes_list: List of joints to home.
        """
        if axes_list is None:
            axes_list = self.robot.hh_axes_list
        self.setup_coordmv()
        if hasattr(self.robot, 'shdeg'):
            self.coordmv(self.anglestoirc(np.array(self.robot.shdeg)))
        else:
            self.coordmv(self.anglestoirc(np.array(self.robot.hhdeg)))
        self.wait_ready()

    def query(self, query):
        """
        Send query to control unit.
        :param query: Query to send.
        :return: Control unit's response.
        """
        buf = '\n'
        self.send_cmd('\n' + query + '?\n')
        while True:
            buf += self.read_resp(1024)
            i = buf.find('\n' + query + '=')
            if i < 0:
                continue
            j = buf[i+1:].find('\n')
            if j != -1:
                break
        if buf[i+1+j-1] == '\r':
            j -= 1
        res = buf[i+2+len(query):i+1+j]
        return res

    def command(self, command):
        """
        Send command to control unit.
        :param command: Command to send
        """
        self.send_cmd(command + ':\n')

    def find_closest_ikt(self, pos, prev_pos=None):
        """
        Find closest configuration to previous position of home position.
        :param pos: Coordinates of position to move to, specified in world coordinates.
        :param prev_pos: Previous position of robot.
        :return: Coordinates of closest position in IRC or None if there isn't any position reachable
        """
        a = self.robot.ikt(self.robot, pos)
        num = None
        min_dist = float('Inf')

        if hasattr(self.robot, 'shdeg'):
            prev_pos = self.anglestoirc(np.array(self.robot.shdeg)) if prev_pos is None else prev_pos
        else:
            prev_pos = self.anglestoirc(np.array(self.robot.hhdeg)) if prev_pos is None else prev_pos
        for i in range(len(a)):
            irc = self.anglestoirc(a[i])
            validm = irc > self.robot.bound[0]
            validp = irc < self.robot.bound[1]
            valid = np.logical_and(validm, validp)
            if np.all(valid):
                dist = np.linalg.norm(np.array(irc) - prev_pos)
                if dist < min_dist:
                    min_dist = dist
                    num = i
        if num is None:
            raise ValueError("Position is unreachable!")
        irc = self.anglestoirc(a[num])

        return irc

    def move_to_pos(self, irc, relative=False):
        """
        Move robot to coordinates stated in IRC using coordinated movement.
        :param irc: joint coordinates in IRC
        :param relative: Boolean, whether the movement is relative to previous position.
        """
        if self.last_trgt_irc is None:
            relative = False
        if relative:
            prev_irc = self.last_trgt_irc
            irc = list(irc - prev_irc)
        self.coordmv(irc, relative=relative)

    def wait_ready(self, sync=False):
        """
        Wait for control unit to be ready.
        :param sync: Boolean, whether to synchronize with control unit.
        """
        buf = '\n'
        if sync:
            self.sync_cmd_fifo()
            print('Synchronized!')
        self.send_cmd("\nR:\n")
        while True:
            buf += self.read_resp(1024)
            if buf.find('\nR!') >= 0:
                return True
            if buf.find('\nFAIL!') >= 0:
                return False

    def wait_gripper_ready(self):
        """
        Wait for gripper to be ready.
        """
        if not hasattr(self.robot, 'gripper_ax'):
            raise Exception('This robot has no gripper_ax defined.')

        self.send_cmd('\nR%s:\n'%self.robot.gripper_ax)
        self.rcon.timeout = 2

        s = self.read_resp(1024)
        self.rcon.timeout = 0.01
        if s.find('R%s!\r\n' % self.robot.gripper_ax) >= 0:
            last = float('inf')
            while True:
                self.send_cmd('AP%s?\n'%self.robot.gripper_ax)
                s = self.read_resp(1024)

                if s.find('\nFAIL!') >= 0:
                    raise Exception('Command \'AP\' returned \'FAIL!\n')
                ifs = s.find('AP%s=' % self.robot.gripper_ax)
                if ifs >= 0:
                    ifl = s.find('\r\n')
                    p = float(s[ifs+4:ifl])
                if abs(last - p) < self.robot.gripper_poll_diff:
                    break
                last = p
            time.sleep(self.robot.gripper_poll_time/100)

        elif s.find('\nFAIL!') >= 0:
            self.wait_ready()
            raise Exception('Command \'R:%s\' returned \'FAIL!\''%self.robot.gripper_ax)

    def open_comm(self, tty_dev, speed=19200):
        """
        Open serial communication port.
        :param tty_dev: Device to open.
        :param speed: Baud rate of serial communication.
        """
        print("Opening %s ...\n" % tty_dev)
        ser = serial.Serial(tty_dev,
                            baudrate=speed,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            rtscts=True,
                            timeout=0.01)

        self.set_rcon(ser)
        self.init_communication()

    def release(self):
        """
        Release errors and reset control unit.
        """
        self.send_cmd("RELEASE:\n")

    def init(self, **kwargs):
        """
        Initialize commander. Initialize robot and perform homing.
        :param kwargs: Parameters for initialisation.
        """
        self.init_robot()
        reg_type = kwargs.get('reg_type', None)
        max_speed = kwargs.get('max_speed', None)
        hard_home = kwargs.get('hard_home', True)

        if reg_type is not None:
            self.send_cmd("RELEASE:\n")
            self.set_int_param_for_axes(param='REGTYPE', val=reg_type)

        if max_speed is not None:
            self.set_max_speed(val=max_speed)

        if hard_home:
            print("Running home")
            self.hard_home()
            self.soft_home()
            print("Hard and soft home done!")

    def reset_motors(self):
        """
        Reset motors of robot.
        """
        self.send_cmd("PURGE:\n")
