import argparse
import os
import time
import cv2
import numpy as np
from robotics_toolbox.core import SE3, SO3
from vision import Camera, Detector
from runner_utils import Robot


def get_calib_data():
    OK = 0
    ERROR = 0
    tty_dev = r"/dev/ttyUSB0"
    detector = Detector(False)
    camera = Camera(False)
    robot = Robot(tty_dev, True)
    camera_cords_list = []
    dkt_cord_list = []
    X = np.linspace(525, 700, 5)
    Y = np.linspace(-200, 50, 5)
    Z = np.linspace(100, 700, 12)
    #angle_z = np.linspace(-)
    for z in Z:
        for x in X:
            for y in Y:
                try:
                    xyz = [x, y, z, 0, 0, 0]
                    robot.move_xyz(xyz)
                    path = f"data/{str([x, y, z, 0, 0, 0])}z.png"
                    print(path)
                    camera.save_img(path)
                    OK += 1
                except:
                    ERROR +=1
            print(f"SUM: {OK+ERROR}, OK{OK}, ERROR{ERROR}")


    # x = +700  +350
    # y = -200  +60
    # z = +700 +100


def tbm(vector):
    #[0.54878762 - 0.10620403  1.22170883]
    #[-0.37403561  0.3633731 - 0.01518165]
    t = vector[:3]
    r_vec = np.array(vector[3:])
    t = np.array(t) * 1000
    r =  SO3.exp(r_vec)
    transform = SE3(t, r)
    T = SE3([548.78762, -106.20403,  1221.70883], SO3.exp([np.pi,  0, 0]))
    trans = T * transform
    t_vec = trans.translation
    r_vec = trans.rotation.log()
    return np.array([t_vec, r_vec]).flatten()

def tbc(cam_data, dkt_data):
    cam_t = cam_data[:3]
    cam_r = cam_data[3:] #np.flip(cam_data[3:], axis = 0)
    dkt_t = np.array(dkt_data[:3]) / 1000
    angle_x, angle_y, angle_z = np.array(dkt_data[3:])/180 * np.pi
    dkt_r = SO3.rx(angle_x) * SO3.ry(angle_y) * SO3.rz(angle_z)
    T_cam = SE3(cam_t, SO3.exp(cam_r))
    T_dkt = SE3(dkt_t, dkt_r)
    return T_dkt * T_cam.inverse()

def load_data():
    detector = Detector(False)
    img_list = os.listdir("data")
    transforms = []
    OK = 0
    for img in img_list:
        image = cv2.imread(f"./data/{img}")
        str_name = img.split("z")[0].replace('[', '').replace(']', '')
        #print(img)

        dkt_data = np.array(np.fromstring(str_name, dtype=float, sep=','))
        #print(dkt_data)
        cam_data, ids = detector.get_all(image)
        if cam_data is not None and len(cam_data) > 0:
            transform = tbc(cam_data[0], dkt_data)
            transforms.append(transform)
            OK += 1
    print(f"OK {OK/len(img_list)}")
    t_vec = [0, 0, 0]
    r_vec = [0, 0, 0]
    for trans in transforms:
        t_vec += trans.translation
        r_vec += trans.rotation.log()
    t_vec = np.array(t_vec) / len(transforms)
    r_vec = np.array(r_vec) / len(transforms)
    print(t_vec)
    print(r_vec)

    return transforms


def boundaries_test():
    tty_dev = r"/dev/ttyUSB0"
    robot = Robot(tty_dev, False)
    detector = Detector(False)
    camera = Camera(True)
    print("blabla")
    xyz = [600, -0, 100, - 3, 0, 0]
    robot.move_xyz(xyz)
    img = camera.get_image()
    #x = +700  +350
    #y = -200  +60
    #z = +700 +100


def cam():
    camera = Camera(False)
    detector = Detector(True)
    img = camera.get_image()
    bricks, ids = detector.get_all(img)
    print(f"bricks: {bricks}")
    print(f"ids: {ids}")


if __name__ == "__main__":
    cam()

