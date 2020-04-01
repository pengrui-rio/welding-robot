#!/usr/bin/env python
# -*- coding: UTF-8 -*-



#"/home/rick/.local/lib/python2.7/site-packages/urx/robot.py"

import sys  
import cv2
import numpy as np
import time
import sys
import copy
import rospy
from std_msgs.msg import String


def talker():
    rospy.init_node('send_capture_command', anonymous=True)
    pub = rospy.Publisher('send_capture_command', String, queue_size=10)
    rate = rospy.Rate(1000)    #hz

    while not rospy.is_shutdown():
        print "\n============ Press `Enter` to send_capture_command ============"
        raw_input()

        str = "capture_pointcloud!"  
        rospy.loginfo(str)
        pub.publish(str)
        rate.sleep()

        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass