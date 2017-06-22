#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import Adafruit_PCA9685
import time
import picamera
import datetime
import os
from Adafruit_BNO055 import BNO055

# PWM setup
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

def pic_cb(data):

    global n_img, curr_gas, curr_dir, drive, bno

    x, y, z = bno.read_linear_acceleration()
    image_name = save_folder + 'frame_'+ str(n_img) + '_gas_' + str(curr_gas) + '_dir_'
    image_name += str(curr_dir) +"_xacc_" + str(x) + "_yacc_" + str(y) + "_zacc_" + str(z) +'_' + '.jpg'
    cam.capture(image_name, quality=10, resize=(250,150))
    img_nb += 1

def dir_cb(data):

    global curr_dir, commands

    curr_dir = data.data

    if curr_dir == 0:
        pwm.set_pwm(commands['direction'], 0 , commands['straight'])
    else:
        pwm.set_pwm(commands['direction'], 0 , int(curr_dir * (commands['right'] - commands['left'])/2. + commands['straight']))


def gas_cb(data):
    global curr_gas, commands

    curr_gas = data.data

    if commands['reverse']:
        if curr_gas < 0:
            pwm.set_pwm(commands['gas'], 0 , commands['rev_neutral'])
        elif curr_gas == 0:
            pwm.set_pwm(commands['gas'], 0 , commands['rev_neutral'])
        else:
            pwm.set_pwm(commands['gas'], 0 , int(curr_gas * (commands['rev_drive_max']-commands['rev_drive']) + commands['rev_drive']))
    else:
        if curr_gas < 0:
            pwm.set_pwm(commands['gas'], 0 , commands['stop'])
        elif curr_gas == 0:
            pwm.set_pwm(commands['gas'], 0 , commands['neutral'])
        else:
            pwm.set_pwm(commands['gas'], 0 , int(curr_gas * (commands['drive_max']-commands['drive']) + commands['drive']))


def change_dir_cb(data):
    global commands

    reverse = commands['reverse']

    change_direction = data.data
    if change_direction:
        reverse = not reverse
        if reverse:
            pwm.set_pwm(commands['gas'], 0, 350)
            time.sleep(1)
            pwm.set_pwm(commands['gas'], 0, commands['rev_neutral'])
        else:
            pwm.set_pwm(commands['gas'], 0, 390)
            time.sleep(1)
            pwm.set_pwm(commands['gas'], 0, commands['neutral'])
    commands['reverse'] = reverse


def callback(data):
    global curr_gas, curr_dir, commands

    received_command = data.data
    rospy.loginfo("just received: %s", received_command)

    if received_command == "left":
        pwm.set_pwm(commands['direction'], 0 , commands['left'])
        curr_dir = -1
    elif received_command == "right":
        pwm.set_pwm(commands['direction'], 0 , commands['right'])
        curr_dir = 1
    elif received_command in ["rightreleased", "leftreleased"]:
        pwm.set_pwm(commands['direction'], 0 , commands['straight'])
        curr_dir = 0
    elif received_command == "up":
        pwm.set_pwm(commands['gas'], 0 , commands['drive'])
        curr_gas = 0.5
    elif received_command == "down":
        pwm.set_pwm(commands['gas'], 0 , commands['stop'])
        curr_gas = 0
    elif received_command in ["upreleased", "downreleased"]:
        pwm.set_pwm(commands['gas'], 0 , commands['neutral'])
        curr_gas = 0

def initialize_motor():
    global step, commands, bno, xacc_threshold

    x = 0
    for k in range(0, 50, step):
        print(x)
        pwm.set_pwm(2, 0 , 360 + k)
        x, y, z = bno.read_linear_acceleration()
        time.sleep(0.3)
        if x > xacc_threshold:
            print("detected movement, setting straight to ", 360+k)
            pwm.set_pwm(2, 0 , commands['neutral'])
            time.sleep(2)
            drive = 360 + k + init_gas
            commands['drive'] = drive
            break

class ArgumentError(Exception):
    print('Error in arguments !')


def main():

    global controls

    rospy.loginfo("Launching listener")

    rospy.init_node('drive', anonymous=True)

    if controls in ['kb', 'keyboard']:
        print('Using keyboard')
        rospy.Subscriber("dir_gas", String, callback)
        rospy.Subscriber("pic", String, pic_cb)
        print("Ready, entering loop")
        rospy.spin()
    elif controls in ['gp', 'gamepad']:
        print('Using gamepad')
        rospy.init_node('drive', anonymous=True)
        rospy.Subscriber("dir", Float32, dir_cb)
        rospy.Subscriber("gas", Float32, gas_cb)
        rospy.Subscriber("pic", String, pic_cb)
        rospy.Subscriber("rev", Bool, change_dir_cb)
    elif controls in ['auto', 'autopilot']:
        print('Running in autopilot !')
        # TODO



if __name__ == '__main__':

    possible_arguments = ['-c', '--controls', '-i', '--init-motor',
                            '-s', '--save-folder']
    arguments = sys.argv[1:]

    controls = 'gamepad'
    init_motor = False
    # TODO
    commands = {'direction': 1, 'left': 310, 'right': 490, 'straight': 400,
                'gas': 2, 'drive': 400, 'stop': 200, 'neutral': 360,
                'drive_max': 420, 'reverse': False, 'rev_neutral': 380,
                'rev_drive': 370, 'rev_drive_max': 360, 'rev_stop': 400}

    ct = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    save_folder = str(ct) + '/'

    # Camera setup
    cam = picamera.PiCamera()
    n_img = 0

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

    curr_dir = 0
    curr_gas = 0

    i = 0
    while i < len(arguments):
        arg = arguments[i]
        if arg not in possible_arguments:
            raise ArgumentError
        if arg in ['-c', '--controls']:
            if i+1 >= len(arguments):
                raise ArgumentError
            controls = arguments[i+1]
            if controls not in ['k', 'keyboard', 'gd', 'gamepad', 'auto', 'autopilot']:
                print('These controls do not exist')
                raise ArgumentError
            i += 1
        if arg in ['-i', '--init-motor']:
            init_motor = True
        if arg in ['-s', '--save-file']:
            if i+1 >= len(arguments):
                print('No save file has been given.')
                print('Using the default one : ', save_folder)
            else:
                save_folder = arguments[i+1]
            if not os.path.exists(save_folder):
                os.makedirs(save_folder)
            i += 1
        i += 1

    # IMU setup
    xacc_threshold = 0.2
    bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

    if init_motor:
        initialize_motor()

    main()
