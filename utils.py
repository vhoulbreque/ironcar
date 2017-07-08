import os
import json


def initialize_imu(loop=5):

    from Adafruit_BNO055 import BNO055

    # IMU setup
    bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
    for i in range(0, loop):
        try:
            if not bno.begin():
                raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
            else:
                print("Initialized IMU correctly")
                break
        except:
            if i == 4 :
                print("Failed initializing IMU {} times, aborting. Please check the IMU connection".format(loop))
            else:
                print("Failed initializing IMU, trying again")

    return bno


def load_controls(path):

    if not os.path.isfile(path):
        raise IOError

    controls = json.loads(''.join([l for l in open(path, 'r')]))
    return controls


class ArgumentError(Exception):

    def __init__(self, placeholder=None):
        self.placeholder = placeholder

    def __str__(self):
        display_error_text = 'ArgumentError '
        if self.placeholder is not None:
            display_error_text += self.placeholder
        print(display_error_text)
        return self

class FolderExistsError(OSError):

    def __init__(self, placeholder=None):
        self.placeholder = placeholder

    def __str__(self):
        display_error_text = 'FolderExistsError '
        if self.placeholder is not None:
            display_error_text += self.placeholder
        print(display_error_text)
        return self
