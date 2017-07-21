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
import scipy.misc

from Adafruit_BNO055 import BNO055
from utils import ArgumentError, initialize_imu


def main(mode):

    print('Loading {} mode'.format(mode))

    if mode == 'autopilot':

        # cam setup
        fps = 60
        cam = picamera.PiCamera(framerate=fps)
        cam.resolution = (250, 150)
        cam_output = picamera.array.PiRGBArray(cam, size=(250, 150))
        stream = cam.capture_continuous(cam_output, format="rgb", use_video_port=True)
        # ros publisher setup
        image_pub = rospy.Publisher("/camera", CompressedImage,  queue_size=2)
        rospy.init_node('image_pub', anonymous=True)

        # ind = 0 
        for f in stream:
            x, y, z = 0,0,0  #bno.read_linear_acceleration()
            acc = str(x) + "_" + str(y) + "_" + str(z) + "_"
            
            img_arr = f.array

            msg = CompressedImage()
            msg.format = "jpeg"

            msg.header.stamp = rospy.Time.now()
            msg.data = img_arr.tostring()
            msg.header.frame_id = acc
            image_pub.publish(msg)
            cam_output.truncate(0)
            # print(ind)
            # ind += 1 

    elif mode == 'training':

        def pic_talker():
            pub = rospy.Publisher('pic', String, queue_size=10)
            rospy.init_node('pic_talker', anonymous=True)
            rate = rospy.Rate(10) # 10hz
            while not rospy.is_shutdown():
                hello_str = "hi"
                pub.publish(hello_str)
                rate.sleep()

        pic_talker()

    else:
        print('This mode ({}) does not exist'.format(mode))


if __name__ == '__main__':

    possible_arguments = ['-t', '--training', '-a', '--autopilot']
    arguments = sys.argv[1:]

    mode = 'training'
    bno = None
    xacc_threshold = 0.2

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
        bno = initialize_imu(5)

    main(mode)
