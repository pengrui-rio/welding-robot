#!/usr/bin/env python
# -*- coding: UTF-8 -*-





import numpy as np
import time
import sys
import copy
import rospy
from geometry_msgs.msg import PoseStamped, Pose
import logging
from math import pi
import numpy as np
import urx
import math3d as m3d

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
    p.append(pose.orientation.w)

    motion_pathPoint.append( p )
    print p 
    print len(motion_pathPoint)
    print "\n" 



if __name__ == "__main__":
    try:  
        logging.basicConfig(level=logging.WARN)

        robot = urx.Robot("192.168.0.2")

        rospy.init_node('robot_motion', anonymous=True)

        rospy.Subscriber("Welding_Trajectory", Pose, callback_path)
    
        pub = rospy.Publisher('robot_currentpose', PoseStamped, queue_size=10)
        rate = rospy.Rate(1000) # 1000hz

        pose = robot.getl()
        print pose
        
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

        print "============ Press `Enter` to start execution ============"
        raw_input()
        # v = 0.05
        # a = 0.01


        # pose = robot.getl()
        # pose[0] = -0.0229312
        # pose[1] = -0.61587
        # pose[2] =  0.208105 + 0.005
        # pose[3] =  0.186891  
        # pose[4] =  2.88628
        # pose[5] = -1.32473
        
        # robot.movep(pose, acc=a, vel=v, wait=False)
        # while True:
        #     p = robot.getl(wait=False)
        #     if abs(p[0] - pose[0]) < 0.001 and abs(p[1] - pose[1]) < 0.001 and abs(p[2] - pose[2]) < 0.001:
        #         print "26"
        #         break

        # pose = robot.getl()
        # pose[0] = -0.0262489 
        # pose[1] = -0.606981
        # pose[2] =  0.208293 + 0.005
        # pose[3] =  0.146154  
        # pose[4] =  2.87789
        # pose[5] = -1.3177
        
        # robot.movep(pose, acc=a, vel=v, wait=False)
        # while True:
        #     p = robot.getl(wait=False)
        #     if abs(p[0] - pose[0]) < 0.001 and abs(p[1] - pose[1]) < 0.001 and abs(p[2] - pose[2]) < 0.001:
        #         print "25"
        #         break

        # pose = robot.getl()
        # pose[0] = -0.0249306 
        # pose[1] = -0.602791
        # pose[2] =  0.2081 + 0.005
        # pose[3] =  0.158271     
        # pose[4] =  2.87836
        # pose[5] = -1.32066
        
        # robot.movep(pose, acc=a, vel=v, wait=False)
        # while True:
        #     p = robot.getl(wait=False)
        #     if abs(p[0] - pose[0]) < 0.001 and abs(p[1] - pose[1]) < 0.001 and abs(p[2] - pose[2]) < 0.001:
        #         print "24"
        #         break

        # pose = robot.getl()
        # pose[0] = -0.0265872 
        # pose[1] = -0.596186
        # pose[2] =  0.20858 + 0.005
        # pose[3] =  0.143093        
        # pose[4] =  2.87408
        # pose[5] = -1.31263
        
        # robot.movep(pose, acc=a, vel=v, wait=False)
        # while True:
        #     p = robot.getl(wait=False)
        #     if abs(p[0] - pose[0]) < 0.001 and abs(p[1] - pose[1]) < 0.001 and abs(p[2] - pose[2]) < 0.001:
        #         print "23"
        #         break

        # pose = robot.getl()
        # pose[0] = -0.0321295   
        # pose[1] = -0.78747
        # pose[2] =  0.208412 + 0.005
        # pose[3] =  0.0995896            
        # pose[4] =  2.85721
        # pose[5] = -1.30472
        
        # robot.movep(pose, acc=a, vel=v, wait=False)
        # while True:
        #     p = robot.getl(wait=False)
        #     if abs(p[0] - pose[0]) < 0.001 and abs(p[1] - pose[1]) < 0.001 and abs(p[2] - pose[2]) < 0.001:
        #         print "22"
        #         break

        # pose = robot.getl()
        # pose[0] = -0.0427129
        # pose[1] = -0.551723
        # pose[2] =  0.208434 + 0.005
        # pose[3] =  -0.0249529                  
        # pose[4] =   3.13478
        # pose[5] = -2.58956
        
        # robot.movep(pose, acc=a, vel=v, wait=False)
        # while True:
        #     p = robot.getl(wait=False)
        #     if abs(p[0] - pose[0]) < 0.001 and abs(p[1] - pose[1]) < 0.001 and abs(p[2] - pose[2]) < 0.001:
        #         print "21"
        #         break




        # l = 0.05
        # v = 0.01
        # a = 0.01
        # pose = robot.getl()

        # pose[2] += l
        # robot.movep(pose, acc=a, vel=v, wait=False)
        # while True:
        #     p = robot.getl(wait=False)
        #     if abs(p[2] - pose[2]) < 0.005:
        #         print "break"
        #         break

        # pose[1] += l 
        # robot.movep(pose, acc=a, vel=v, wait=False)
        # while True:
        #     p = robot.getl(wait=False)
        #     if abs(p[1] - pose[1]) < 0.005:
        #         print "break"
        #         break

        # pose[2] -= l
        # robot.movep(pose, acc=a, vel=v, wait=False)
        # while True:
        #     p = robot.getl(wait=True)
        #     if abs(p[2] - pose[2]) < 0.005:
        #         break

        # pose[1] -= l
        # robot.movep(pose, acc=a, vel=v, wait=False)


        # theta = np.sqrt(pose[3]*pose[3]+pose[4]*pose[4]+pose[5]*pose[5])
        # print theta
        # print pose[3] / theta
        # print pose[4] / theta
        # print pose[5] / theta
        # print("absolute move in base coordinate ")

    # finally:
    #     robot.close()
    # try:
    #     # l = 0.02
        # v = 0.01
        # a = 0.01
        # pose = rob.getl()
        # print("robot tcp is at: ", pose)
        # print("absolute move in base coordinate ")
        # pose[0] += l
        # rob.movel(pose, acc=a, vel=v)
        # print("relative move in base coordinate ")
        # rob.translate((0, 0, -l), acc=a, vel=v)
        # print("relative move back and forth in tool coordinate")
        # rob.translate_tool((0, 0, -l), acc=a, vel=v)
        # rob.translate_tool((0, 0, l), acc=a, vel=v)

        # mytcp = m3d.Transform()
        # mytcp.pos.z = 0
        # mytcp.orient.w = 0
        # robot.set_tcp(mytcp)
        # # rob.set_tcp((0,0,0,0,0,0))
        # # rob.set_payload(1, (0,0,0))

        # trans = robot.get_pose()  # get current transformation matrix (tool to base)
        # trans.pos.z += 0.06
        # trans.orient.rotate_yb(0)
        # robot.set_pose(trans, acc=0.01, vel=0.01)  # apply the new pose

        # trans = robot.get_pose()  # get current transformation matrix (tool to base)
        # trans.pos.x += 0.06
        # trans.orient.rotate_yb(0)
        # robot.set_pose(trans, acc=0.01, vel=0.01)  # apply the new pose


    finally:
        robot.close()