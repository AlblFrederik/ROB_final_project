import time
import numpy as np
from robot_utils import *



H = [[-6.86788605e-01, 2.27244915e-02, 4.90485726e+02],
     [-4.26111153e-02, 7.60420347e-01, -6.69100595e+01],
     [2.10283255e-04, 9.15293985e-05, 1.00000000e+00]]

H2 = [[-9.63240103e-01, -3.98482820e-02, 4.84493445e+02],
      [-2.72601751e-02, 9.65478422e-01, -6.80515124e+01],
      [-2.85583138e-05, -2.53222255e-05, 1.00000000e+00]]

H3 = [[-9.67752019e-01, -3.44604189e-02, 4.91843407e+02],
      [-1.97610670e-02, 9.63380155e-01, -6.83824958e+01],
      [-9.76010465e-06, -2.57165898e-06, 1.00000000e+00]]


class Brick:

    def __init__(self, position, status, brick_id):
        self.position = position
        self.status = status
        self.brick_id = brick_id


class Robot:

    def __init__(self, tty_dev, do_init=False):
        self.robot = robCRS93()
        self.commander = Commander(self.robot)  # initialize commander
        self.commander.open_comm(tty_dev)  # connect to control unit
        self.do_init = do_init
        self.cords_q = None
        self.cord_xyz = None
        self.gripper = None
        self.last_i = 0
        self.last_len = None
        self.rep = 0
        self.brick_list = []
        self.__enter__()

    def __enter__(self):
        if self.do_init is True:
            max_speed = None
            reg_type = None
            self.commander.reset_motors()
            self.commander.wait_ready()
            self.commander.init(reg_type=reg_type, max_speed=max_speed, hard_home=True, home=False)
            self.commander.wait_ready()
        elif self.do_init == 1:
            max_speed = None
            reg_type = None
            self.commander.init(reg_type=reg_type, max_speed=max_speed, hard_home=False, home=True)
            self.commander.wait_ready()

    @staticmethod
    def equal_position(cords1, cords2):
        if np.linalg.norm(np.array(cords1[:2]) - np.array(cords2[:2])) <= 0.02:
            return True
        return False

    def update_brick_list(self, bricks, ids):
        if len(self.brick_list) == 0:
            for i in range(len(bricks)):
                self.brick_list.append(Brick(bricks[i], "unknown", ids[i]))
        else:
            for i in range(len(bricks)):
                exists = False
                for j in range(len(self.brick_list)):
                    if self.equal_position(bricks[i], self.brick_list[j].position):
                        exists = True
                if not exists:
                    self.brick_list.append(Brick(bricks[i], "unknown", ids[i]))

    def update_brick_object(self, brick, status):
        for br in self.brick_list:
            if self.equal_position(brick, br.position):
                br.status = status

    def select_brick(self, bricks, ids):
        self.update_brick_list(bricks, ids)
        brick, brick_id = None, None
        for br in self.brick_list:
            if br.status is "unknown":
                brick = br.position
                brick_id = br.brick_id
        return brick, brick_id

    def pick_up_brick(self, cords_xyz):
        above_cords = np.array(cords_xyz) + np.array([0, 0, 50, 0, 0, 0])
        ret = self.move_xyz(above_cords)
        if ret:
            self.open_gripper()
            ret = self.move_xyz(cords_xyz)
        if ret:
            self.close_gripper()
            ret = self.move_xyz(above_cords)  # lift brick
        return ret

    def lay_down_brick(self, cords_xyz):
        if cords_xyz[2] < 53:
            cords_xyz[2] = 53
        if cords_xyz[2] > 150:
            cords_xyz[2] = 100

        above_cords = np.array(cords_xyz) + np.array([0, 0, 50, 0, 0, 0])
        ret = self.move_xyz(above_cords)
        if ret:
            ret = self.move_xyz(cords_xyz)
        if ret:
            self.open_gripper()
            ret = self.move_xyz(above_cords)  # lift brick
        return ret

    def go_to_box(self, cords_xyz=None):
        if cords_xyz is None:
            cords_xyz = [100, -400, 200, 0, 90, 0]  # default box position
        ret = self.move_xyz(np.array(self.cord_xyz) + [0, 0, 100, 0, 0, 0])
        if ret:
            if cords_xyz[2] < 150:
                cords_xyz[2] = 150
            ret = self.move_xyz(cords_xyz)
        if ret:
            self.open_gripper()
            ret = self.move_xyz(np.array(cords_xyz) + [0, 0, 50, 0, 0, 0])
        return ret

    def get_xyz_dkt(self, cords_q):
        dkt_res = robCRSdkt(self.robot, cords_q)
        return dkt_res

    def open_gripper(self):
        robCRSgripper(self.commander, -0.2)
        time.sleep(1)
        self.gripper = 0

    def close_gripper(self):
        robCRSgripper(self.commander, 1)
        time.sleep(1)
        self.gripper = 1

    def lock(self):
        self.commander.release()

    def move_xyz(self, position):
        try:
            irc = self.commander.find_closest_ikt(position)
            if irc is None:
                print("No config")
                return False
            self.commander.move_to_pos(irc)
            self.commander.wait_ready()
            self.cord_xyz = position
            return True
        except:
            return False

    @staticmethod
    def get_brick_in_bot_cords(cam_cords):
        if cam_cords is not None:
            r_vec = cam_cords[3:]
            brick = np.array(cam_cords) * 1000
            transformed = H3 @ np.append(brick[:2], [1])
            an = r_vec[2]
            transformed[0] = transformed[0] - 0 * np.cos(an / 180 * np.pi)
            transformed[1] = transformed[1] - 10 * np.sin(an / 180 * np.pi)
            to_move = np.hstack((transformed, np.array([0, 90, an])))
            to_move[2] = 53
            return to_move
        return None
