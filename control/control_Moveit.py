#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import numpy as np
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

 
def euler_to_quaternion(Yaw, Pitch, Roll):
  yaw   = Yaw   * pi / 180 
  pitch = Roll  * pi / 180 
  roll  = Pitch * pi / 180 

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]


def all_close(goal, actual, tolerance):
 
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonInteface(object):
 
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()
 
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_motion', anonymous=True)

    rospy.Subscriber("Moveit_motion_Path", Pose, self.callback_path)
 
    pub = rospy.Publisher('robot_currentpose_Moveit', PoseStamped, queue_size=10)
    rate = rospy.Rate(1000) # 1000hz


    robot = moveit_commander.RobotCommander()
 
    scene = moveit_commander.PlanningSceneInterface()
 
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
 
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
 
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
 
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
 
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
 
    # print "============ Printing robot state"
    print robot.get_current_state()
 

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.pub = pub
    self.rate = rate
    self.motion_pathPoint = []
    self.marker_info = 1
 


  def callback_path(self, pose):
    p = []
    p.append(pose.position.x)
    p.append(pose.position.y)
    p.append(pose.position.z)
    p.append(pose.orientation.x)
    p.append(pose.orientation.y)
    p.append(pose.orientation.z)
    p.append(pose.orientation.w)

    self.motion_pathPoint.append( p )
    print p 
    print len(self.motion_pathPoint)
    print "\n" 


  def motion(self):

    print "============ Press `Enter` to set the initial pose ..."
    raw_input()

    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/2
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = 0
 
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
 

    print "============ Press `Enter` to capture pointcloud ============"
    raw_input()
    group = self.group

    x = 0.0
    y = 0.4#0.51   
    z = 0.15#0.25

    roll  = 135  #右手坐标系 静态欧拉角旋转方向： R-X-red -> P-Y-green -> Y-Z-purple 以base_link为基准旋转
    pitch = 0     
    yaw   = 0         

    pose_goal = geometry_msgs.msg.Pose()
    Q = euler_to_quaternion(yaw , roll, pitch)
    pose_goal.orientation.x = Q[0]
    pose_goal.orientation.y = Q[1]
    pose_goal.orientation.z = Q[2]
    pose_goal.orientation.w = Q[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()


    # while not rospy.is_shutdown():
    #   print "============ Press `Enter` to capture pointcloud ============"
    #   raw_input()
    #   pub_pose = PoseStamped()
    #   pub_pose.header.stamp       = rospy.Time.now()
    #   pub_pose.header.frame_id    = "robot_currentpose"
    #   pub_pose.pose.position.x    = 0
    #   pub_pose.pose.position.y    = 0
    #   pub_pose.pose.position.z    = 0
    #   pub_pose.pose.orientation.x = 0
    #   pub_pose.pose.orientation.y = 0
    #   pub_pose.pose.orientation.z = 0
    #   pub_pose.pose.orientation.w = 2
    #   rospy.loginfo(pub_pose)
    #   self.pub.publish(pub_pose)

##############################################################################################################
    # print "============ Press `Enter` to capture pointcloud ============"
    # raw_input()
    # group = self.group
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.x = -0.993773990796
    # pose_goal.orientation.y = 0.0168818280301
    # pose_goal.orientation.z = 0.00422386723625
    # pose_goal.orientation.w = 0.110047344564
    # pose_goal.position.x    = 0.0535405512804
    # pose_goal.position.y    = 0.475299484695
    # pose_goal.position.z    = 0.386624271589 
    # group.set_pose_target(pose_goal)

    # plan = group.go(wait=True)
    # group.stop()
    # #################################################################


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
    self.pub.publish(pub_pose)



    # print "============ Press `Enter` to start execution ============"
    # raw_input()

    # group = self.group
    # joint_goal = group.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = -pi/2
    # joint_goal[2] = 0
    # joint_goal[3] = -pi/2
    # joint_goal[4] = 0
    # joint_goal[5] = 0

    # group.go(joint_goal, wait=True)
    # group.stop()
    # group.clear_pose_targets()
    # current_pose = self.group.get_current_pose().pose


    # group = self.group 
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x    = 0.03942855820059776
    # pose_goal.position.y    = 0.7659310698509216
    # pose_goal.position.z    = 0.023764735087752342
    # pose_goal.orientation.x = 0.9951673126324553
    # pose_goal.orientation.y = -0.0978338734221161
    # pose_goal.orientation.z = -0.0076631620494124674
    # pose_goal.orientation.w = -0.003442025858636122
    # group.set_pose_target(pose_goal)

    # plan = group.go(wait=True)
    # group.stop()

 
    # move_dir_flag = 1
    # i = 0
    # while not rospy.is_shutdown():
    #   if move_dir_flag == 1:
    #     group = self.group 

    #     pose_goal = geometry_msgs.msg.Pose()
    #     pose_goal.position.x    = self.motion_pathPoint[i][0] 
    #     pose_goal.position.y    = self.motion_pathPoint[i][1]
    #     pose_goal.position.z    = self.motion_pathPoint[i][2]
    #     pose_goal.orientation.x = self.motion_pathPoint[i][3]
    #     pose_goal.orientation.y = self.motion_pathPoint[i][4]
    #     pose_goal.orientation.z = self.motion_pathPoint[i][5]
    #     pose_goal.orientation.w = self.motion_pathPoint[i][6]
    #     group.set_pose_target(pose_goal)
    #     group.go(wait=True)
    #     group.stop()
    #     print i

    #     pub_pose = PoseStamped()
    #     pub_pose.header.stamp       = rospy.Time.now()
    #     pub_pose.header.frame_id    = "robot_currentpose"
    #     pub_pose.pose.position.x    = 0
    #     pub_pose.pose.position.y    = 0
    #     pub_pose.pose.position.z    = 0
    #     pub_pose.pose.orientation.x = 0
    #     pub_pose.pose.orientation.y = 0
    #     pub_pose.pose.orientation.z = 0
    #     pub_pose.pose.orientation.w = 3
    #     rospy.loginfo(pub_pose)
    #     self.pub.publish(pub_pose)

    #     i = i + 1
    #     if i == len(self.motion_pathPoint) - 1 + 1:
    #       move_dir_flag = -1

    #   elif move_dir_flag == -1:
    #     group = self.group 

    #     pose_goal = geometry_msgs.msg.Pose()
    #     pose_goal.position.x    = self.motion_pathPoint[i][0] 
    #     pose_goal.position.y    = self.motion_pathPoint[i][1]
    #     pose_goal.position.z    = self.motion_pathPoint[i][2]
    #     pose_goal.orientation.x = self.motion_pathPoint[i][3]
    #     pose_goal.orientation.y = self.motion_pathPoint[i][4]
    #     pose_goal.orientation.z = self.motion_pathPoint[i][5]
    #     pose_goal.orientation.w = self.motion_pathPoint[i][6]
    #     group.set_pose_target(pose_goal)

    #     plan = group.go(wait=True)
    #     group.stop()

    #     i = i - 1
    #     if i == 0 - 1:
    #       move_dir_flag = 1
    # print "============ Press `Enter` to start execution ============"
    # raw_input()

    # group = self.group
    # joint_goal = group.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = -pi/2
    # joint_goal[2] = 0
    # joint_goal[3] = -pi/2
    # joint_goal[4] = 0
    # joint_goal[5] = 0

    # group.go(joint_goal, wait=True)
    # group.stop()
    # group.clear_pose_targets()
    # current_pose = self.group.get_current_pose().pose
 

    # while not rospy.is_shutdown():
    #   waypoints = []
    #   wpose = geometry_msgs.msg.Pose(); i = 0
    #   while i < len(self.motion_pathPoint):
    #     wpose.position.x    = self.motion_pathPoint[i][0] 
    #     wpose.position.y    = self.motion_pathPoint[i][1]
    #     wpose.position.z    = self.motion_pathPoint[i][2]
    #     wpose.orientation.x = self.motion_pathPoint[i][3]
    #     wpose.orientation.y = self.motion_pathPoint[i][4]
    #     wpose.orientation.z = self.motion_pathPoint[i][5]
    #     wpose.orientation.w = self.motion_pathPoint[i][6]
    #     print wpose
    #     print "\n"

    #     waypoints.append(copy.deepcopy(wpose))
    #     i = i + 1

    #   print waypoints
    #   (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0)   
    #   # raw_input()

    #   result_plan = group.retime_trajectory(self.robot.get_current_state(), plan, 1)
    #   group.execute(result_plan, wait=True)
    #   # ///////////////////////////////////////////////////////
    #   waypoints = []
    #   wpose = geometry_msgs.msg.Pose(); i = 0
    #   while i < len(self.motion_pathPoint):
    #     wpose.position.x    = self.motion_pathPoint[len(self.motion_pathPoint) - 1 - i][0] 
    #     wpose.position.y    = self.motion_pathPoint[len(self.motion_pathPoint) - 1 - i][1]
    #     wpose.position.z    = self.motion_pathPoint[len(self.motion_pathPoint) - 1 - i][2]
    #     wpose.orientation.x = self.motion_pathPoint[len(self.motion_pathPoint) - 1 - i][3]
    #     wpose.orientation.y = self.motion_pathPoint[len(self.motion_pathPoint) - 1 - i][4]
    #     wpose.orientation.z = self.motion_pathPoint[len(self.motion_pathPoint) - 1 - i][5]
    #     wpose.orientation.w = self.motion_pathPoint[len(self.motion_pathPoint) - 1 - i][6]
    #     print wpose
    #     print "\n"

    #     waypoints.append(copy.deepcopy(wpose))
    #     i = i + 1

    #   print waypoints
    #   (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0)   
    #   # raw_input()

    #   result_plan = group.retime_trajectory(self.robot.get_current_state(), plan, 1)
    #   group.execute(result_plan, wait=True)

    # #################################################################
    # print "============ Back to initial status ============"
    # result_plan = group.retime_trajectory(self.robot.get_current_state(), plan, 1)
    # group = self.group
    # joint_goal = group.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = -pi/2
    # joint_goal[2] = 0
    # joint_goal[3] = -pi/2
    # joint_goal[4] = 0
    # joint_goal[5] = 0
 
    # group.go(joint_goal, wait=True)
    # group.stop()
    # group.clear_pose_targets()
    # current_pose = self.group.get_current_pose().pose

    return all_close(pose_goal, current_pose, 0.01)


 
 


#robot_ip = 192.168.0.2
def main():
  try:
    print "============ Press `Enter` to start configuration ..."
    manipulator = MoveGroupPythonInteface()
 
 
    manipulator.motion()
 

    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
 












