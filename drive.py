#!/usr/bin/env python

"""

drive.py receives control commands (from keyboard, gamepad or autopilot,
handled by remote_controller) and gives input to the car.

Launched on the Raspberry Pi

"""

import sys
import rospy
import numpy as np
import scipy.misc
from std_msgs.msg import String, Float32, Bool
import Adafruit_PCA9685
import time
import picamera
import picamera.array
import datetime
import os
from Adafruit_BNO055 import BNO055
from utils import ArgumentError, initialize_imu

def pic_cb(data):

    global n_img, curr_gas, curr_dir, drive, bno, save_folder, cam, cam_output, stream

    x, y, z = bno.read_linear_acceleration()
    image_name = os.path.join(save_folder, 'frame_'+ str(n_img) + '_gas_' +
                                str(curr_gas) + '_dir_' + str(curr_dir) +
                                "_xacc_" + str(x) + "_yacc_" + str(y) +
                                "_zacc_" + str(z) +'_' + '.jpg')
    for f in stream:
        im_arr = f.array
        break
    cam_output.truncate(0)
    im_arr = np.array(im_arr[80:, :, :], copy=True)
    scipy.misc.imsave(image_name, im_arr)
    n_img += 1

def dir_cb(data):

    global curr_dir, commands, pwm

    curr_dir = data.data
    if curr_dir == 0:
        print(commands['straight'])
        pwm.set_pwm(commands['direction'], 0 , commands['straight'])
    else:
        print(int(curr_dir * (commands['right'] - commands['left'])/2. + commands['straight']))
        pwm.set_pwm(commands['direction'], 0 , int(curr_dir * (commands['right'] - commands['left'])/2. + commands['straight']))


def gas_cb(data):
    global curr_gas, commands, reverse

    curr_gas = data.data
    if reverse:
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
    global commands, reverse

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


def keyboard_cb(data):
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


def initialize_motor(xacc_threshold):
    global step, commands, bno, init_gas

    if bno is None: return

    x = 0
    neutral = commands['neutral']
    for k in range(0, 50, step):
        gas = neutral + k
        pwm.set_pwm(2, 0 , gas)
        x, y, z = bno.read_linear_acceleration()
        time.sleep(0.3)
        print('acceleration (x_acc) detected : {} with a gas set at : {}'.format(x, gas))
        if x > xacc_threshold:
            pwm.set_pwm(2, 0 , commands['neutral'])
            drive = neutral + k + init_gas
            commands['drive'] = drive
            print('detected movement, setting `drive` to ', drive)
            time.sleep(2)
            break

def main():

    global controls

    rospy.loginfo("Launching listeners")

    if controls == 'keyboard':
        print('Using keyboard')
        rospy.init_node('drive', anonymous=True)
        rospy.Subscriber("dir_gas", String, keyboard_cb)
        rospy.Subscriber("pic", String, pic_cb)
        print("Ready, entering loop")
        rospy.spin()
    elif controls == 'gamepad':
        print('Using gamepad')
        rospy.init_node('drive', anonymous=True)
        rospy.Subscriber("dir", Float32, dir_cb)
        rospy.Subscriber("gas", Float32, gas_cb)
        rospy.Subscriber("pic", String, pic_cb)
        rospy.Subscriber("rev", Bool, change_dir_cb)
        print("Ready, entering loop")
        rospy.spin()
    elif controls == 'autopilot':
    	print('Running in autopilot !')
        rospy.loginfo("Launching listener")
        rospy.init_node('drive', anonymous=True)
        rospy.Subscriber("dir", Float32, dir_cb)
        rospy.Subscriber("gas", Float32, gas_cb)
        print("Ready, entering loop")
        rospy.spin()

    else:
        print('No controller has been set: \n'
              'Current controller is {}'.format(controls))


if __name__ == '__main__':

    possible_arguments = ['-k', '--keyboard',
                          '-a', '--autopilot',
                          '-g', '--gamepad',
                          '-s', '--save-folder',
                          '--no-imu',
                          '--no-init-motor',
                          '--no-init']

    arguments = sys.argv[1:]

    controls = 'keyboard'
    init_motor = True
    init_imu = True

    commands = {'direction': 1, 'left': 310, 'right': 490, 'straight': 400,
                'gas': 2, 'drive': 400, 'stop': 210, 'neutral': 385,
                'drive_max': 420, 'rev_neutral': 370,
                'rev_drive': 360, 'rev_drive_max': 350, 'rev_stop': 400,
                'go_t': 0.25, 'stop_t': -0.25, 'left_t': 0.5, 'right_t': -0.5}

    ct = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    save_folder = os.path.join('videos/', str(ct))

    n_img = 0

    # PWM setup
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(60)

    cam = None
    cam_output = None

    # init_motor
    step = 2
    init_gas = 10

    curr_dir = 0
    curr_gas = 0
    reverse = False

    #IMU params
    xacc_threshold = 0.2
    bno = None

    i = 0
    while i < len(arguments):
        arg = arguments[i]
        if arg not in possible_arguments:
            raise ArgumentError
        elif arg in ['--no-imu']:
            init_imu = False
        elif arg in ['-k', '--keyboard']:
            controls = 'keyboard'
        elif arg in ['-g', '--gamepad']:
            controls = 'gamepad'
        elif arg in ['-a', '--autopilot']:
            controls = 'autopilot'
        elif arg in ['--no-init-motor']:
            init_motor = False
        elif arg in ['--no-init']:
            init_motor = False
            init_imu = False
        elif arg in ['-s', '--save-file']:
            if i+1 >= len(arguments):
                print('No save file has been given.')
                print('Using the default one : ', save_folder)
            else:
                save_folder = arguments[i+1]
            i += 1
        i += 1

    if init_imu:
        bno = initialize_imu(5)

    if init_motor:
        initialize_motor(xacc_threshold)

    # cam setup
    if controls != 'autopilot':
        print('Setting up the pi camera')
        cam = picamera.PiCamera(framerate=60)
        cam.resolution = (250, 150)
        cam_output = picamera.array.PiRGBArray(cam, size=(250, 150))
        stream = cam.capture_continuous(cam_output, format="rgb", use_video_port=True)
        print('Pi camera set up')

    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    main()
