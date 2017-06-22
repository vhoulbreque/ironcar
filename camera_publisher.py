#!/usr/bin/env python

import sys
import time
import os
import numpy as np

import roslib
import rospy
from sensor_msgs.msg import CompressedImage

import picamera
import picamera.array

from Adafruit_BNO055 import BNO055

# IMU setup
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
for i in range(0,5):
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
    x,y,z = bno.read_linear_acceleration()
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
