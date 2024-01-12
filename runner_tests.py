from runner_utils import *
from vision import *

# H = [[ -9.33813763e-01,  -1.79260102e-02,   4.61504297e+02],
#  [ -4.72404223e-02,   9.51363143e-01,  -6.88594285e+01],
#  [  3.11412840e-05,  1.77463171e-05,   1.00000000e+00]]

H = [[ -9.21277451e-01,  -4.30118588e-02,   4.65879225e+02],
 [ -4.00458432e-02,   9.54258461e-01,  -6.77511315e+01],
 [  7.01636300e-05,  -3.30854449e-05,   1.00000000e+00]]


def move_above():
    tty_dev = r"/dev/ttyUSB0"
    robot = Robot(tty_dev, False)
    robot.move_xyz([600, -100, 200, 0, 0, 0])
    camera = Camera(False)
    detector = Detector(True)
    img = camera.get_image()
    bricks, ids = detector.get_all(img)
    brick = select_brick(bricks)
    if brick is not None:
        brick = np.array(brick) * 1000
        print(f"brick {brick}")
        transfromed = H @ np.append(brick[:2], [1])
        print(transfromed)
        to_move = np.hstack((transfromed, np.array([0, 90, 0])))
        to_move[2] = 300
        print(to_move)
        # robot.move_xyz([460.67630223,  -68.94153812,  300.0,            0.0,            0.0,            0.0])
        robot.move_xyz(to_move)


if __name__ == "__main__":
    move_above()