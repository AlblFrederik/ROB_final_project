import time

import PyCapture2
import cv2
import numpy as np
from PIL import Image

# Initialize bus and camera
bus = PyCapture2.BusManager()
camera = PyCapture2.Camera()

# Select first camera on the bus
camera.connect(bus.getCameraFromIndex(0))

# Start capture
camera.startCapture()

while True:
    # Retrieve image from camara in PyCapture2.Image format
    image = camera.retrieveBuffer()

    # Convert from MONO8 to RGB8
    image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)

    # Convert image to Numpy array
    rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3));

    # Convert RGB image to BGR image to be shown by OpenCV
    bgr_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_RGB2BGR)

    # Show image
    cv2.imshow('frame', bgr_cv_image)

    # Wait for key press, stop if the key is q
    if cv2.waitKey(1) & 0xFF == ord('p'):
        cv2.imwrite("calibrations/img{}.jpg".format(time.time()), bgr_cv_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break