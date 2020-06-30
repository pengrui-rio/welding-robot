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
# import math3d as m3d
import math
import csv



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

    print rotationVector[0]
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


def read_csv(path):
    csv_info = []

    csvFile = open(path, "r")
    reader = csv.reader(csvFile)

    for item in reader:
        csv_info.append(item)

    csvFile.close()

    motiontrajectory = []
    for i in range(len(csv_info)):
        if len(csv_info[i]) == 0:
            print "empty"
            continue

        p = []
        p.append(float(csv_info[i][0]))
        p.append(float(csv_info[i][1]))
        p.append(float(csv_info[i][2]))
        p.append(float(csv_info[i][3]))
        p.append(float(csv_info[i][4]))
        p.append(float(csv_info[i][5]))
        motiontrajectory.append(p)
        print p

    return motiontrajectory


def write_csv(PC_path, trajectoryInfo):
    csvFile = open(PC_path, "w")
    writer = csv.writer(csvFile)
    for i in range(len(trajectoryInfo)):
        writer.writerow(trajectoryInfo[i])
    csvFile.close()


def inverse_trajectory(motiontrajectory):
    inverse_motiontrajectory = []
    for i in range(len(motiontrajectory)):
        pose = []
        pose = motiontrajectory[len(motiontrajectory) - 1 - i]       

        inverse_motiontrajectory.append(pose)

    return inverse_motiontrajectory


motion_pathPoint = []

def callback_path(pose):
    global motion_pathPoint

    p = []
    p.append(pose.position.x)
    p.append(pose.position.y)
    p.append(pose.position.z)
    p.append(pose.orientation.x)
    p.append(pose.orientation.y)
    p.append(pose.orientation.z)

    motion_pathPoint.append( p )
    print p 
    print len(motion_pathPoint)
    print "\n" 


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

    robot.movel(pose, acc=0.005, vel=0.06, wait=True)



def trajectory_execution(robot, pose_list):
    print "\n============ Press `Enter` to execute welding trajectory and record trajectory info ============"
    raw_input()


    move_to_singlePose(robot, pose_list[0])

    robot.movels(pose_list, acc=0.01, vel=0.02, wait=False)




if __name__ == "__main__":
    try:  
        logging.basicConfig(level=logging.WARN)
        robot = urx.Robot("192.168.0.2")

        rospy.init_node('robot_motion', anonymous=True)
        rospy.Subscriber("Welding_Trajectory", Pose, callback_path)

        pub = rospy.Publisher('robot_currentpose', PoseStamped, queue_size=10)
        rate = rospy.Rate(1000) # 1000hz

        print "\n"
        pose = robot.getl()######################################################tch, yaw
        # move_to_singleXYZRPY(robot, 0,  -0.3, 0.35, 90, 180, 0)  #x, y, z, roll, pitch, yaw
        # goto_straight_status(robot)


        # #########################################################################################################
        move_to_singleXYZRPY(robot, 0, -0.3, 0.5, 90, 180, 0)  #x, y, z, roll, pitch, yaw

        move_to_singleXYZRPY(robot, 0, -0.3, 0.5,  0, 180, 0)  #x, y, z, roll, pitch, yaw


        # # print "wait"
        # raw_input()
        # write_csv("/home/rick/Documents/a_system/src/motion_planning1executing/traj_control_execution/trajectoryURx_csv/Y-shape.csv", motion_pathPoint)


 

        # motiontrajectory = read_csv("/home/rick/Documents/a_system/src/motion_planning1executing/traj_control_execution/trajectoryURx_csv/test.csv")
        # inverse_motiontrajectory = inverse_trajectory(motiontrajectory)
        # print "\n"
        # print motiontrajectory

        # while not rospy.is_shutdown():
        #     print "\n"
        #     print motiontrajectory
        #     trajectory_execution(robot, motiontrajectory)

        #     time.sleep(2)

        #     print "\n"
        #     print inverse_motiontrajectory
        #     trajectory_execution(robot, inverse_motiontrajectory)

        #     time.sleep(2)



        ##########################################################################################################

    finally:
        robot.close() 














        # motiontrajectory = read_csv("/home/rick/Documents/a_system/src/motion_planning1executing/traj_control_execution/trajectoryURx_csv/cad_test.csv")
        # print "\n"
        # print motiontrajectory
        # trajectory_execution(robot, motiontrajectory)


        # pose = []
        # index = 1
        # pose.append(motiontrajectory[index-1][0])
        # pose.append(motiontrajectory[index-1][1])
        # pose.append(motiontrajectory[index-1][2])
        # pose.append(motiontrajectory[index-1][3])
        # pose.append(motiontrajectory[index-1][4])
        # pose.append(motiontrajectory[index-1][5])
        # move_to_singlePose(robot, pose)

        # pose = []
        # index = 2
        # pose.append(motiontrajectory[index-1][0])
        # pose.append(motiontrajectory[index-1][1])
        # pose.append(motiontrajectory[index-1][2])
        # pose.append(motiontrajectory[index-1][3])
        # pose.append(motiontrajectory[index-1][4])
        # pose.append(motiontrajectory[index-1][5])
        # move_to_singlePose(robot, pose)



