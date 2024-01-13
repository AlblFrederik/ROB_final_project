from runner_utils import *
from vision import *


def move_above():
    tty_dev = r"/dev/ttyUSB0"
    robot = Robot(tty_dev, True)
    robot.move_xyz([100, -400, 200, 0, 90, 0])
    camera = Camera(False)
    detector = Detector(True)
    img = camera.get_image()
    bricks, ids = detector.get_all(img)
    brick = robot.select_brick(bricks, ids)

    if brick is not None:
        to_move = robot.get_brick_in_bot_cords(brick)
        print(f"to move: {to_move}")
        to_move[2] = 90
        robot.pick_up_brick(to_move)
    # robot.lay_down_brick([550, -150, 50, 0, 90, 0])
    robot.lock()


def soft():
    tty_dev = r"/dev/ttyUSB0"
    robot = Robot(tty_dev, True)


def pickup():
    tty_dev = r"/dev/ttyUSB0"
    robot = Robot(tty_dev, False)
    robot.move_xyz([100, 400, 200, 0, 90, 0])
    robot.lock()


if __name__ == "__main__":
    # soft()
    pickup()
    # move_above()

