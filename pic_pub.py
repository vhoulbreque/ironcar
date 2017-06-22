#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def pic_talker():
    pub = rospy.Publisher('pic', String, queue_size=10)
    rospy.init_node('pic_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hi"
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

pic_talker()
