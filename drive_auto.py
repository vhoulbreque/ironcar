#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import Adafruit_PCA9685
import time
import datetime
import os
from Adafruit_BNO055 import BNO055

# IMU setup
xacc_threshold = 0.5
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
    
    
# PWM setup
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

left = 310
right = 490
straight = 400
drive = 400
stop = 200
neutral = 380
direction = 1
gas = 2
curr_dir = 0
curr_gas = 0
step = 2
init_gas = 10

running = True

def callback(data):
    global curr_gas, curr_dir, straight
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo("just received: %s",data.data)
    #print('data.data : ', data.data)
    if data.data == "left":
        pwm.set_pwm(direction, 0 , left)
        curr_dir = -1
    elif data.data == "right":
        pwm.set_pwm(direction, 0 , right)
        curr_dir = 1
    elif data.data == "rightreleased" or data.data == "leftreleased":
        pwm.set_pwm(direction, 0 , straight)
        curr_dir = 0
    elif data.data == "up":
        print(drive)
        print('+'*92)
        pwm.set_pwm(gas, 0 , drive)
        curr_gas = 0.5
    elif data.data == "down":
        pwm.set_pwm(gas, 0 , stop)
        curr_gas = 0
    elif data.data == "upreleased" or data.data == "downreleased":
        pwm.set_pwm(gas, 0 , neutral)
        curr_gas = 0

def init_motor():
    global straight, drive
    x = 0
    time.sleep(1) # wait for the imu to initialize
    for k in range(0, 50, step):
        pwm.set_pwm(2, 0 , 360 + k)
        x,y,z = bno.read_linear_acceleration()
        print("Initializing car, wait for it to boot. Current x acceleration:" + str(x))
        time.sleep(0.3)
        if abs(x) > xacc_threshold:
            print("detected movement, setting gas to ", 360+k)
            pwm.set_pwm(2, 0 , neutral)
            time.sleep(5)
            drive = 360 + k + init_gas
            if drive < 390: drive = 400
            drive = 395
            break

init_motor()
rospy.loginfo("Launching listener")
rospy.init_node('drive', anonymous=True)
rospy.Subscriber("dir_gas", String, callback)
print("Ready, entering loop")
rospy.spin()
