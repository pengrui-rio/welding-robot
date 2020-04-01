#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import numpy as np
import time
import sys
import copy
import rospy
import geometry_msgs.msg
from math import pi
from sensor_msgs.msg import Imu


def callback_imu(imu):
	print "imu.angular_velocity:"
	print imu.angular_velocity
	print "imu.linear_acceleration:"
	print imu.linear_acceleration
	print "\n"
	

if __name__ == '__main__':
	rospy.init_node('imu_process', anonymous=True)

	rospy.Subscriber("imu0", Imu, callback_imu)

	print "start"

	while not rospy.is_shutdown():
		time.sleep(0.01)
