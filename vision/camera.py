try:
    import time
    import PyCapture2
    import cv2
    import numpy as np
except:
    print("Import error in camera.py")


class Camera:

    def __init__(self, visualize=True):
        self.visualize = visualize

    def get_image(self, visualize=False):
        bus = PyCapture2.BusManager()
        cam = PyCapture2.Camera()
        # Select first camera on the bus
        cam.connect(bus.getCameraFromIndex(0))
        # Start capture
        cam.startCapture()
        # Retrieve image from camara in PyCapture2.Image format

        time.sleep(0.2)
        image = cam.retrieveBuffer()

        # Convert from MONO8 to RGB8
        image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)

        # Convert image to Numpy array
        rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))

        # Convert RGB image to BGR image to be shown by OpenCV
        bgr_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_RGB2BGR)
        if self.visualize:
            cv2.imshow('frame', bgr_cv_image)
            cv2.waitKey()
        cam.stopCapture()
        cam.disconnect()
        return bgr_cv_image

    @staticmethod
    def save_img(path):
        bus = PyCapture2.BusManager()
        cam = PyCapture2.Camera()
        # Select first camera on the bus
        cam.connect(bus.getCameraFromIndex(0))
        # Start capture
        cam.startCapture()
        # Retrieve image from camara in PyCapture2.Image format
        time.sleep(0.2)
        image = cam.retrieveBuffer()

        # Convert from MONO8 to RGB8
        image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)

        # Convert image to Numpy array
        rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3));

        # Convert RGB image to BGR image to be shown by OpenCV
        bgr_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_RGB2BGR)
        path = str(path)
        try:
            cv2.imwrite(path, bgr_cv_image)
            cam.stopCapture()
            cam.disconnect()
            return True
        except Exception as E:
            print("Camera failed to save image: {}".format(E))
            cam.stopCapture()
            cam.disconnect()
            return False

    @staticmethod
    def stream():
        print("quit by q")
        # Initialize bus and camera
        bus = PyCapture2.BusManager()
        cam = PyCapture2.Camera()
        # Select first camera on the bus
        cam.connect(bus.getCameraFromIndex(0))
        # Start capture
        cam.startCapture()
        while True:
            # Retrieve image from camara in PyCapture2.Image format
            image = cam.retrieveBuffer()

            # Convert from MONO8 to RGB8
            image = image.convert(PyCapture2.PIXEL_FORMAT.RGB8)

            # Convert image to Numpy array
            rgb_cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3));

            # Convert RGB image to BGR image to be shown by OpenCV
            bgr_cv_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_RGB2BGR)
            # Show image
            cv2.imshow('frame', bgr_cv_image)

            # Wait for key press, stop if the key is q
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cam.stopCapture()
                cam.disconnect()
                break