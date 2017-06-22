import Adafruit_PCA9685
import picamera
import picamera.array
import time
import sys
import numpy as np

from keras.models import load_model


def initialize_motor(init_gas, step):
    print('Initialialiazing the motor...')
    global commands
    x = 0
    for k in range(0, 50, step):
        print(x)
        pwm.set_pwm(gas, 0 , 360 + k)
        x, y, z = bno.read_linear_acceleration()
        time.sleep(0.3)
        if x > xacc_threshold:
            print("detected movement, setting straight to ", 360+k)
            pwm.set_pwm(gas, 0 , commands['neutral'])
            time.sleep(2)
            drive = 360 + k + init_gas
            commands['drive'] = drive
            break
    print('Motor initialized !')

def main(model, cam_output, commands):

    go_on = True

    drive_t, stop_t, left_t, right_t = commands['go'], commands['stop'], commands['left'], commands['drive']
    direction, left, right, straight = commands['direction'], commands['left'], commands['right'], commands['straight']
    gas, drive, stop, neutral = commands['gas'], commands['drive'], commands['stop'], commands['neutral']

    while go_on:
        cam.capture(cam_output, 'rgb', quality=10, resize=(250,150))
        img = np.array(cam_output.array)

        commands = model.predict(img)

        c0, c1 = commands[0], commands[1]
        # gas
        if c0 > drive_t: curr_gas = drive
        elif c0 < stop_t: curr_gas = stop
        else: curr_gas = neutral
        # dir
        if c1 > right_t: curr_dir = right
        elif c1 < left_t: curr_dir = left
        else: curr_dir = straight

        print('current direction: ', curr_dir, ' current gas: ', curr_gas)

        pwm.set_pwm(gas, 0 , curr_gas)
        pwm.set_pwm(direction, 0 , curr_dir)


class ArgumentError(Exception):
    print('Error in arguments !')


if __name__ == '__main__':

    possible_arguments = ['-m', '--model', '-i', '--init-motor',
                            '-c', '--commands-folder']
    arguments = sys.argv[1:]

    model_path = 'autopilot_2.hdf5'
    init_motor = False
    commands = {'go_t': 0.25, 'stop_t': -0.25, 'left_t': 0.5, 'right_t': -0.5,
                    'direction': 1, 'left': 310, 'right': 490, 'straight': 400,
                    'gas': 2, 'drive': 400, 'stop': 200, 'neutral': 360}

    # cam setup
    print('Setting up the pi camera')
    cam = picamera.PiCamera()
    cam_output = picamera.array.PiRGBArray(cam)
    print('Pi camera set up')

    # PWM setup
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(60)

    # init_motor
    step = 2
    init_gas = 10

    i = 0
    while i < len(arguments):
        arg = arguments[i]
        if arg not in possible_arguments:
            raise ArgumentError
        if arg in ['-m', '--model']:
            if i+1 >= len(arguments):
                raise ArgumentError
            model_path = arguments[i+1]
            if not os.path.isfile(model_path):
                print('This path does not exist : {}'.format(model_path))
                raise ArgumentError
            i += 1
        if arg in ['-i', '--init-motor']:
            init_motor = True
        if arg in ['-c', '--commands-folder']:
            if i+1 >= len(arguments):
                raise ArgumentError
            command_file = arguments[i+1]
            if not os.isfile(command_file):
                print('No command_file found at : ', command_file)
                print('Using default thresholds')
            else:
                commands = load_commands(command_file)
            i += 1
        i += 1

    model = load_model(model_path)

    if init_motor:
        initialize_motor(init_gas, step)

    main(model, cam_output, commands)
