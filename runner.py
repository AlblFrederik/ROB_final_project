from vision import *
from runner_utils import *


case = 0
LOCATION_COUNTER = 0

tty_dev = r"/dev/ttyUSB0"
camera = Camera(visualize=False)
detector = Detector(visualize=False)
robot = Robot(tty_dev=tty_dev, do_init=1)
robot.close_gripper()

brick = [400, 0, 300, 0, 0, 0]
box1 = [50, -500, 100, 0, 90, 0]
box2 = [200, -450, 100, 0, 90, 0]
box = [200, 500, 100, 0, 90, 0]
ids = []
id1 = None
id2 = None
brick_id = None

while True:

    if case == 0:
        robot.move_xyz([100, -400, 300, 0, 90, 0])
        img = camera.get_image()
        bricks, ids = detector.get_all(img)
        if bricks is None or len(bricks) == 0:
            LOCATION_COUNTER += 1
            if LOCATION_COUNTER > 1:
                robot.lock()
                exit()
        else:
            LOCATION_COUNTER = 0
            brick, brick_id = robot.select_brick(bricks, ids)
            if brick is None:
                robot.lock()
                print("No other options")
                exit()
            case = 1

    if case == 1:
        to_move = robot.get_brick_in_bot_cords(brick)
        ret = robot.pick_up_brick(to_move)
        if ret:
            robot.update_brick_object(brick, "in box")
            case = 2
        else:
            robot.update_brick_object(brick, "unreachable")
            case = 0

        # move above brick
        # pick up brick

    if case == 2:
        # which box
        box = None
        if brick_id is not None:
            if id1 is None:
                id1 = brick_id
            elif id2 is None and brick_id is not id1:
                id2 = brick_id
        if brick_id == id1:
            print(f"{brick_id} => Going to box 1")
            box = box1
        elif brick_id == id2:
            print(f"{brick_id} => Going to box 2")
            box = box2
        else:
            print(f"{brick_id} => Going away")
            box = [100, 400, 200, 0, 90, 0]
        case = 3

    if case == 3:
        ret = robot.go_to_box(box)
        if ret:
            case = 0
