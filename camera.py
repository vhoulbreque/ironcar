#!/usr/bin/env python

import sys
import time
import os
import numpy as np

import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import picamera
import picamera.array

from Adafruit_BNO055 import BNO055


def initialiaze_imu():

    global bno

    # IMU setup
    bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
    for i in range(0, 5):
        try:
            if not bno.begin():
                raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
            else:
                print("Initialized IMU correctly")
                break
        except:
            if i == 4 :
                print("Failed initializing IMU 5 times, aborting. Please check the IMU connection")
            else:
                print("Failed initializing IMU, trying again")

def main():

    global mode
    print('mode : ', mode)
    if mode == 'autopilot':

        # cam setup
        cam = picamera.PiCamera()
        cam_output = picamera.array.PiRGBArray(cam, size=(250,150))

        # ros publisher setup
        image_pub = rospy.Publisher("/camera", CompressedImage, queue_size=130000)
        rospy.init_node('image_pub', anonymous=True)

        msg = CompressedImage()
        msg.format = "jpeg"

        rate = rospy.Rate(10) # 10 Hz

        while not rospy.is_shutdown():
            x, y, z = bno.read_linear_acceleration()
            acc = str(x) + "_" + str(y) + "_" + str(z) + "_"
            cam.capture(cam_output, 'rgb', resize=(250,150))
            img_arr = np.array([cam_output.array])
            msg.header.stamp = rospy.Time.now()
            print(img_arr.shape)
            msg.data = img_arr.tostring()
            msg.header.frame_id = acc
            image_pub.publish(msg)
            cam_output.truncate(0)
            rate.sleep()

    elif mode == 'training':

        def pic_talker():
            pub = rospy.Publisher('pic', String, queue_size=10)
            rospy.init_node('pic_talker', anonymous=True)
            rate = rospy.Rate(10) # 10hz
            while not rospy.is_shutdown():
                hello_str = "hi"
                print(hello_str)
                #rospy.loginfo(hello_str)
                pub.publish(hello_str)
                rate.sleep()

        pic_talker()


if __name__ == '__main__':

    possible_arguments = ['-t', '--training', '-a', '--autopilot']
    arguments = sys.argv[1:]

    mode = 'training'
    bno = None

    i = 0
    while i < len(arguments):
        arg = arguments[i]
        if arg not in possible_arguments:
            raise ArgumentError
        if arg in ['-a', '--autopilot']:
            mode = 'autopilot'
        elif arg in ['-t', '--training']:
            mode = 'training'
        i += 1

    if mode == 'autopilot':
        initialiaze_imu()
    
    main()
