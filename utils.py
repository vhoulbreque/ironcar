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


def load_commands(path):

    if not os.path.isfile(path):
        raise IOError

    lines = [l for l in open(path, 'r')]

    new_lines = []
    for line in lines:
        words = line.split('#')
        nl = words[0].replace('\'', '"')
        if '\n' not in nl:
            nl += '\n'
        new_lines.append(nl)

    commands = json.loads(''.join(new_lines))
    return commands


class ArgumentError(Exception):

    def __init__(self):
        pass

    def __str__(self):
        print('Error in arguments !')
        return self
