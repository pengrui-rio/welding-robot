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

    robot.movel(pose, acc=0.01, vel=0.06, wait=True)



def trajectory_execution(robot, pose_list):
    print "\n============ Press `Enter` to execute welding trajectory ============"
    raw_input()

    move_to_singlePose(robot, pose_list[0])

    robot.movels(pose_list, acc=0.01, vel=0.005, wait=False)




if __name__ == "__main__":
    try:  
        logging.basicConfig(level=logging.WARN)
        robot = urx.Robot("192.168.0.2")

        rospy.init_node('robot_motion', anonymous=True)
        rospy.Subscriber("Welding_Trajectory", Pose, callback_path)

        pub = rospy.Publisher('robot_currentpose', PoseStamped, queue_size=10)
        rate = rospy.Rate(1000) # 1000hz

        print "\n"
        pose = robot.getl()
        print  pose
        print "\n"
        # time.sleep(5)
        ##########################################################################################################

        goto_straight_status(robot)

         ##########################################################################################################
    
        # move_to_singleXYZRPY(robot, 0, -0.35, 0.65, 45, 180, 0) #x, y, z, roll, pitch, yaw

        # move_to_singleXYZRPY(robot, 0, -0.35, 0.4, 90, 180, 0)  #x, y, z, roll, pitch, yaw
        move_to_singleXYZRPY(robot, 0, -0.4, 0.52, 0, 180, 0)  #x, y, z, roll, pitch, yaw



        ##########################################################################################################

        # publish_message(rospy, pub)

        # ##########################################################################################################
        p = []
        p.append(-0.33581450571370174)
        p.append(-0.5828907257651846) 
        p.append(0.2376662487989381)
        p.append(0.4000576056466147 )
        p.append(2.0482721844335616)
        p.append(-0.8624739020543665)
        move_to_singlePose(robot, p)
        #1
        p = []
        p.append(-0.23581450571370174)
        p.append(-0.5828907257651846) 
        p.append(0.2376662487989381)
        p.append(0.4000576056466147 )
        p.append(2.0482721844335616)
        p.append(-0.8624739020543665)
        motion_pathPoint.append( p )
        #2
        p = []
        p.append(-0.2221146899927865) 
        p.append(-0.5814885532982753) 
        p.append(0.2553262837838194)
        p.append(0.3931434397984484 )
        p.append(2.1203744328244087)
        p.append(-0.8894410962326792)
        motion_pathPoint.append( p )
        #3
        p = []
        p.append(-0.20366215323725173) 
        p.append(-0.5791885592147559) 
        p.append(0.2744696837559921)
        p.append(0.27107625326123025 )
        p.append(2.210371893060575)
        p.append(-0.9768649365011101)
        motion_pathPoint.append( p )
        #4
        p = []
        p.append(-0.1722236342288759) 
        p.append(-0.5745450521406145) 
        p.append(0.2984900792582126)
        p.append(0.30559734839798525 )
        p.append(2.3373482658821763)
        p.append(-1.020778241124243)
        motion_pathPoint.append( p )
        #5
        p = []
        p.append(-0.12050059544699468) 
        p.append(-0.5649095767684411) 
        p.append(0.3197834278506917)
        p.append(0.24190158593050046 )
        p.append(2.4921734331600076)
        p.append(-1.1563612824384197)
        motion_pathPoint.append( p )
        #6
        p = []
        p.append(-0.10304794196654288) 
        p.append(-0.5614885532982753) 
        p.append(0.3240626397094948)
        p.append(0.2029593580876598 )
        p.append(2.536279210592956)
        p.append(-1.2068149377738868)
        motion_pathPoint.append( p )
        #7
        p = [] 
        p.append(-0.07339322291045422) 
        p.append(-0.557513500432722) 
        p.append(0.33002532668576146)
        p.append(0.20559296983511932)
        p.append(2.6159124981669857)
        p.append(-1.2718667120199045)
        motion_pathPoint.append( p )
        #8
        p = []
        p.append(-0.05180577472005789) 
        p.append(-0.5559180017804217) 
        p.append(0.33301304495298784)
        p.append(0.16877913130492786 )
        p.append(2.6797192344849114)
        p.append(-1.3206739343615257)
        motion_pathPoint.append( p )
        #9
        p = []
        p.append(-0.0330848488251389) 
        p.append(-0.5549799214462767) 
        p.append(0.33448883607391416)
        p.append(0.01933020826332822 )
        p.append(2.741850902953673)
        p.append(-1.3698495276082603)
        motion_pathPoint.append( p )
        #10
        p = []
        p.append(0.005953997489946843) 
        p.append(-0.554020958400903) 
        p.append(0.3344153613039239)
        p.append(-0.03228379419641597 )
        p.append(2.860247400674242)
        p.append(-1.4353575955579059)
        motion_pathPoint.append( p )
        #11
        p = []
        p.append(0.04200003336732733) 
        p.append(-0.5539995612280111) 
        p.append(0.32895701800641697)
        p.append(-0.021759860983094372 )
        p.append(2.9627545603474696)
        p.append(-1.5026835047179539)
        motion_pathPoint.append( p )
        #12
        p = []
        p.append(0.04820034630924353) 
        p.append(-0.5556170400723354) 
        p.append(0.3282998646905027)
        p.append(-0.2664387170728095 )
        p.append(2.9992197098324045)
        p.append(-1.4849220428768646)
        motion_pathPoint.append( p )
        #13
        p = []
        p.append(0.07680586007278353) 
        p.append(-0.5569693321241401) 
        p.append(0.32105312856830737)
        p.append(-0.31329141863492327 )
        p.append(3.0906981681683092)
        p.append(-1.5138660600319667)
        motion_pathPoint.append( p )        
        #14
        p = []
        p.append(0.10435884089459392) 
        p.append(-0.5586803199656423) 
        p.append(0.3113158758114205)
        p.append(-0.3633591670530288 )
        p.append(3.1798469182884546)
        p.append(-1.5435272668304192)
        motion_pathPoint.append( p )
        #15
        p = []
        p.append(0.1303683741073166) 
        p.append(-0.5619030530813516) 
        p.append(0.299722833887339)
        p.append(-0.4840360170305715 )
        p.append(3.27126623403095)
        p.append(-1.5426760332031506)
        motion_pathPoint.append( p )        
        #16
        p = []
        p.append(0.16326750422642766) 
        p.append(-0.5649045746238055) 
        p.append(0.27871148944654073)
        p.append(-0.42051840770418053 )
        p.append(3.3786904638669246)
        p.append(-1.6250699711481584)
        motion_pathPoint.append( p )       
        #17
        p = []
        p.append(0.17990937435711218) 
        p.append(-0.5680799409049243) 
        p.append(0.26609553726939245)
        p.append(-0.5664050450292081 )
        p.append(3.4503954315538383)
        p.append(-1.6053077364668262)
        motion_pathPoint.append( p )
        #18
        p = []
        p.append(0.20007218544671398) 
        p.append(-0.5704681450156541) 
        p.append(0.2450780843672135)
        p.append(-0.7086239275012536 )
        p.append(3.534336567943438)
        p.append(-1.6102576558181034)
        motion_pathPoint.append( p )       
        #19
        p = []
        p.append(0.20797487589255773) 
        p.append(-0.5722998753133436) 
        p.append(0.23669783566738095)
        p.append(-0.8253092311947426 )
        p.append(3.5748225333493036)
        p.append(-1.5739681324162027)
        motion_pathPoint.append( p )
        #20
        p = []
        p.append(0.2359766543296416) 
        p.append(-0.577364860745555) 
        p.append(0.19643939400880706)
        p.append(-0.8929788420063174 )
        p.append(3.716555620817)
        p.append(-1.6570635241187144)
        motion_pathPoint.append( p )       
        #21
        p = []
        p.append(0.24278206518517384) 
        p.append(-0.5784074296609519) 
        p.append(0.18144039048943866)
        p.append(0.4937869987977586 )
        p.append(-1.836909709931889)
        p.append(0.8013142216307765)
        motion_pathPoint.append( p )
        #22
        p = []
        p.append(0.24877076676454257) 
        p.append(-0.578771827127427) 
        p.append(0.16429747379564597)
        p.append(0.5123312831380096 )
        p.append(-1.7888380112164133)
        p.append(0.7786511959583178)
        motion_pathPoint.append( p )

        trajectory_execution(robot, motion_pathPoint)

        p = []
        p.append(0.34877076676454257) 
        p.append(-0.578771827127427) 
        p.append(0.16429747379564597)
        p.append(0.5123312831380096 )
        p.append(-1.7888380112164133)
        p.append(0.7786511959583178)
        move_to_singlePose(robot, p)

        ##########################################################################################################



    finally:
        robot.close()
