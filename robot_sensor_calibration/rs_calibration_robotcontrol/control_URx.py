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



def euler_to_quaternion(Yaw, Pitch, Roll):
  yaw   = Yaw   * pi / 180 
  pitch = Roll  * pi / 180 
  roll  = Pitch * pi / 180 

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]

def euler_to_rotationVector(yaw , roll, pitch):
    nums = euler_to_quaternion(yaw , roll, pitch)
    
    R = np.array([[0,0,0],[0,0,0],[0,0,0]],dtype=float)
    x = float(nums[0])
    y = float(nums[1])
    z = float(nums[2])
    w = float(nums[3]) 

    #calculate element of matrix
    R[0][0] = np.square(w) + np.square(x) - np.square(y) - np.square(z)
    R[0][1] = 2*(x*y + w*z)
    R[0][2] = 2*(x*z - w*y)
    R[1][0] = 2*(x*y - w*z)
    R[1][1] = np.square(w) - np.square(x) + np.square(y) - np.square(z)
    R[1][2] = 2*(w*x + y*z)
    R[2][0] = 2*(x*z + w*y)
    R[2][1] = 2*(y*z - w*x)
    R[2][2] = np.square(w) - np.square(x) - np.square(y) + np.square(z)
    rotationVector  = cv2.Rodrigues(R)

    # print rotationVector[0]
    return rotationVector[0]




def publish_message(rospy, pub):
    print "============ Press `Enter` to sent signal for processing the pointcloud ============"
    raw_input()
    pub_pose = PoseStamped()
    pub_pose.header.stamp       = rospy.Time.now()
    pub_pose.header.frame_id    = "robot_currentpose"
    pub_pose.pose.position.x    = 0
    pub_pose.pose.position.y    = 0
    pub_pose.pose.position.z    = 0
    pub_pose.pose.orientation.x = 0
    pub_pose.pose.orientation.y = 0
    pub_pose.pose.orientation.z = 0
    pub_pose.pose.orientation.w = 1
    rospy.loginfo(pub_pose)
    pub.publish(pub_pose)
    

def goto_straight_status(robot):
    print "\n============ Press `Enter` to go back straight-up status ============"
    raw_input()
    v = 0.3
    a = 0.01

    joint = []
    joint.append(0)
    joint.append(-pi/2)
    joint.append(0)
    joint.append(-pi/2)
    joint.append(0)
    joint.append(0)
    robot.movej(joint, acc=a, vel=v, wait=True)

    print robot.getj()


def move_to_singleXYZRPY(robot, x, y, z, roll, pitch, yaw):
    print "\n============ Press `Enter` to move to certain XYZRPY ============"
    raw_input()

    rotation = euler_to_rotationVector(yaw, roll, pitch) # yaw->roll->pitch
    pose = []
    pose.append(x)              #px
    pose.append(y)              #py
    pose.append(z)              #pz
    pose.append(rotation[0][0]) #rx
    pose.append(rotation[1][0]) #ry
    pose.append(rotation[2][0]) #rz
    robot.movel(pose, acc=0.01, vel=0.06, wait=True)


def move_to_singlePose(robot, pose):
    print "\n============ Press `Enter` to move to certain pose ============"
    raw_input()

    robot.movel(pose, acc=0.01, vel=0.06, wait=True)
 



def automatic_move(robot, x, y, z, roll, pitch, yaw):
 
    rotation = euler_to_rotationVector(yaw, roll, pitch) # yaw->roll->pitch
    pose = []
    pose.append(x)              #px
    pose.append(y)              #py
    pose.append(z)              #pz
    pose.append(rotation[0][0]) #rx
    pose.append(rotation[1][0]) #ry
    pose.append(rotation[2][0]) #rz
    robot.movel(pose, acc=0.001, vel=0.001, wait=False)


marker_Pose = geometry_msgs.msg.Pose()

def callback_marker_info(pose):
    global marker_Pose

    marker_Pose.position.x = pose.position.x
    marker_Pose.position.y = pose.position.y
    marker_Pose.position.z = pose.position.z
 

def automatic_calibration(robot, marker_Pose):
    print "\n============ Press `Enter` to go automatic_calibration ============"
    raw_input()

    i = 0
    while not rospy.is_shutdown():
        i = i + 1
        if i % 10000 == 0:
            i =0
            pose = robot.getl()
            print "pose"
            print pose

        input_x = pose[0] - marker_Pose.position.x
        input_y = pose[1] + marker_Pose.position.y 
        # automatic_move(robot, input_x, input_y, 0.52, 0, 180, 0)
    
    robot.stop()
    print "robot.stop()"



if __name__ == "__main__":
    try:  
        logging.basicConfig(level=logging.WARN)
        robot = urx.Robot("192.168.0.2")

        rospy.init_node('automatic_sensorRobot_calibration', anonymous=True)
        rospy.Subscriber("marker_info", Pose, callback_marker_info)

        rate = rospy.Rate(1000) # 1000hz

        print "\n"
        pose = robot.getl()
        print  pose
        print "\n"
        robot.stop()
        ##########################################################################################################

        # goto_straight_status(robot)

        # ##########################################################################################################
    
        # move_to_singleXYZRPY(robot, 0, -0.35, 0.65, 45, 180, 0) #x, y, z, roll, pitch, yaw

        # move_to_singleXYZRPY(robot, 0, -0.6, 0.52, 90, 180, 0)  #x, y, z, roll, pitch, yaw
        # move_to_singleXYZRPY(robot, -0.03776431571831436, -0.40847125799157285, 0.52, 0, 180, 0)  #x, y, z, roll, pitch, yaw
        move_to_singleXYZRPY(robot, 0, -0.4, 0.65, 0, 180, 0)  #x, y, z, roll, pitch, yaw

        ##########################################################################################################

        automatic_calibration(robot, marker_Pose)



    finally:
        robot.close()
