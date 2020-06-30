#!/usr/bin/env python
# -*- coding: UTF-8 -*-

 
import sys  
sys.path.append('/home/rick/Documents/a_system/src/model_reconstruction/vg_Rcontrol/urx')  
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
# import math3d as m3d
import math
import csv


def Rvector_2_AxisAngle(Base_End_rotation_vector):
    x = Base_End_rotation_vector[0]
    y = Base_End_rotation_vector[1]
    z = Base_End_rotation_vector[2]

    angle = math.sqrt(x*x + y*y + z*z)
    # print angle

    AxisAngle = []
    AxisAngle.append(x / angle)
    AxisAngle.append(y / angle)
    AxisAngle.append(z / angle)
    return AxisAngle


def pointcloud_transform(Base_End, Cam_Point):
    # Base -> End transform matrix:
    Base_End_translation = []
    Base_End_translation.append(Base_End[0])
    Base_End_translation.append(Base_End[1])
    Base_End_translation.append(Base_End[2])

    Base_End_rotation_vector = (Base_End[3],Base_End[4],Base_End[5])
    # Base_End_rotation_vector.append(Base_End[3])
    # Base_End_rotation_vector.append(Base_End[4])
    # Base_End_rotation_vector.append(Base_End[5])
    # AxisAngle = Rvector_2_AxisAngle(Base_End_rotation_vector)
    # print AxisAngle
    Base_End_rotation_matrix = cv2.Rodrigues(Base_End_rotation_vector)[0]
    # print Base_End_rotation_matrix 

    T_B_E = ([Base_End_rotation_matrix[0][0], Base_End_rotation_matrix[0][1], Base_End_rotation_matrix[0][2], Base_End_translation[0]],
             [Base_End_rotation_matrix[1][0], Base_End_rotation_matrix[1][1], Base_End_rotation_matrix[1][2], Base_End_translation[1]],
             [Base_End_rotation_matrix[2][0], Base_End_rotation_matrix[2][1], Base_End_rotation_matrix[2][2], Base_End_translation[2]],
             [0, 0, 0, 1])
    print T_B_E

    # End -> Camera transform matrix
    T_E_C = ([1,            2.7798e-18,            0, -0.0361315],
             [-5.57321e-19,          1,  2.71051e-20,  -0.115202],
             [1.0842e-19,            0,            1,   0.069507],
             [0,                     0,            0,          1])


    # Base -> Camera transform matrix
    T_B_C = np.dot(T_B_E, T_E_C) 
    # print T_B_C


    # Camera -> point transform matrix
    T_C_o = ([1,  0,  0, Cam_Point[0]],
             [0,  1,  0, Cam_Point[1]],
             [0,  0,  1, Cam_Point[2]],
             [0,  0,  0,  1])


    # Transform point to robot base
    T_R = np.dot(T_B_C, T_C_o) 
    print T_R
    object_position_world = []
    object_position_world.append(T_R[0][3])
    object_position_world.append(T_R[1][3])
    object_position_world.append(T_R[2][3])

    return  object_position_world

        
if __name__ == '__main__':
    try:  
        logging.basicConfig(level=logging.WARN)
        robot = urx.Robot("192.168.0.2")

        print "\n"
        # base -> end transformation
        end_pose = robot.getl() 
        print  end_pose
    

        # single point under camera coordinate, eg: x-y-z (0, 0.2, 0.1)
        Cam_Point = []
        Cam_Point.append(0)
        Cam_Point.append(0.2)
        Cam_Point.append(0.1)

        # transform single point to robot base coordinate
        Robotbase_Point = pointcloud_transform(end_pose, Cam_Point)
 


    finally:
        robot.close() 