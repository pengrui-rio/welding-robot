#!/usr/bin/env python
# -*- coding: UTF-8 -*-



#"/home/rick/.local/lib/python2.7/site-packages/urx/robot.py"

import sys  
sys.path.append('../urx/')  
import urx

######################################

import cv2
import numpy as np
import time
import sys
import copy
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
import logging
from math import pi
import numpy as np
import math3d as m3d
import math
 



visuoguding_Pose = geometry_msgs.msg.Pose()

def callback_VisuoGuding_Pose(pose):
    global visuoguding_Pose

    visuoguding_Pose.position.x = pose.position.x
    visuoguding_Pose.position.y = pose.position.y
    visuoguding_Pose.position.z = pose.position.z
    visuoguding_Pose.orientation.x = pose.orientation.x
    visuoguding_Pose.orientation.y = pose.orientation.y
    visuoguding_Pose.orientation.z = pose.orientation.z

    # print visuoguding_Pose 
    # print "\n" 

def VisuoGuding_Control(robot, visuoguding_Pose):
    # print "\n============ Press `Enter` to go VisuoGuding_Control ============"
    # raw_input()
    print_count = 1000
    i = 0
    while not rospy.is_shutdown():
        if i < print_count:
            print i
            print visuoguding_Pose 
            print "\n" 
        i = i + 1
        if i >= print_count:
            i = print_count

        robot_current_pose = robot.getl()
        if math.isnan(visuoguding_Pose.position.x) == False and visuoguding_Pose.position.z >= 0.1:

            pose = []
            pose.append(visuoguding_Pose.position.x)              #px
            pose.append(visuoguding_Pose.position.y)              #py
            pose.append(visuoguding_Pose.position.z)              #pz
            pose.append(robot_current_pose[3]) #rx
            pose.append(robot_current_pose[4]) #ry
            pose.append(robot_current_pose[5]) #rz
            robot.movel(pose, acc=0.01, vel=0.02, wait=True)
    
 

if __name__ == "__main__":
    try:  
        logging.basicConfig(level=logging.WARN)
        robot = urx.Robot("192.168.0.2")

        rospy.init_node('robot_motion', anonymous=True)
        rospy.Subscriber("VisuoGuding_Pose", Pose, callback_VisuoGuding_Pose)
   

        print "\n"
        pose = robot.getl()
        print  pose
        print "\n"
        time.sleep(5)

        ##########################################################################################################

        VisuoGuding_Control(robot, visuoguding_Pose)

        ##########################################################################################################


    finally:
        robot.close()
