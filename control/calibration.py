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

    rospy.Subscriber("marker_info", Pose, self.callback_marker_info)

    pub = rospy.Publisher('robot_currentpose', PoseStamped, queue_size=10)
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



  def callback_marker_info(self, pose):
    self.marker_info = geometry_msgs.msg.Pose()
    self.marker_info.position.x = pose.position.x
    self.marker_info.position.y = pose.position.y
    self.marker_info.position.z = pose.position.z
 

  def robot_sensor_cali(self):
    # print "============ Press `Enter` to set the initial pose ..."
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
 

    print "============ Press `Enter` to capture pointcloud ============"
    raw_input()

    #################################################################
    #对应哪个颜色的柱子，右手坐标系，转多少度，顺序按yaw->pitch->roll,也有很大可能任意顺序
    #yaw->purple   pitch->red   roll->green

    x = 0.0
    y = 0.4
    z = 0.3
           
    yaw   = -0.5       
    pitch = 180               
    roll  =  0  

    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    Q = euler_to_quaternion(yaw , pitch, roll)
    print Q
    pose_goal.orientation.x = Q[0]
    pose_goal.orientation.y = Q[1]
    pose_goal.orientation.z = Q[2]
    pose_goal.orientation.w = Q[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    group.set_pose_target(pose_goal)

    # plan = group.go(wait=True)
    # group.stop()
    group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
    print current_pose.position
    print "yaw   : %f" % yaw
    print "pitch : %f" % pitch
    print "roll  : %f" % roll
    print "\n"

    pub_pose = PoseStamped()
    pub_pose.header.stamp       = rospy.Time.now()
    pub_pose.header.frame_id    = "robot_currentpose"
    pub_pose.pose.position.x    = current_pose.position.x
    pub_pose.pose.position.y    = current_pose.position.y
    pub_pose.pose.position.z    = current_pose.position.z
    pub_pose.pose.orientation.x = current_pose.orientation.x
    pub_pose.pose.orientation.y = current_pose.orientation.y
    pub_pose.pose.orientation.z = current_pose.orientation.z
    pub_pose.pose.orientation.w = current_pose.orientation.w
    rospy.loginfo(pub_pose)
    self.pub.publish(pub_pose)
    #################################################################

    waypoints = []
    wpose = geometry_msgs.msg.Pose(); 
    wpose.orientation.x = 1; wpose.orientation.y = 0; wpose.orientation.z = 0; wpose.orientation.w = 0
    wpose.position.x = 0; wpose.position.y = 0; wpose.position.z = 0 
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0)   
    result_plan = group.retime_trajectory(self.robot.get_current_state(), plan, 0.1)

    print "============ Press `Enter` to 让机械臂自动对准marker中心 ============"
    raw_input()
    while not rospy.is_shutdown():
      motion_interval = 3

      x = current_pose.position.x + self.marker_info.position.x
      y = current_pose.position.y - self.marker_info.position.y
      z = 0.3  
      yaw   = -0.5      
      pitch = 180               
      roll  =  0  

      pose_goal = geometry_msgs.msg.Pose()
      Q = euler_to_quaternion(yaw , pitch, roll)
      pose_goal.orientation.x = Q[0]
      pose_goal.orientation.y = Q[1]
      pose_goal.orientation.z = Q[2]
      pose_goal.orientation.w = Q[3]
      pose_goal.position.x = x
      pose_goal.position.y = y
      pose_goal.position.z = z
      group.set_pose_target(pose_goal)

      # plan = group.go(wait=True)
      # group.stop()
      group.clear_pose_targets()
      current_pose = self.group.get_current_pose().pose
      pub_pose = PoseStamped()
      pub_pose.header.stamp       = rospy.Time.now()
      pub_pose.header.frame_id    = "robot_currentpose"
      pub_pose.pose.position.x    = current_pose.position.x
      pub_pose.pose.position.y    = current_pose.position.y
      pub_pose.pose.position.z    = current_pose.position.z
      pub_pose.pose.orientation.x = current_pose.orientation.x
      pub_pose.pose.orientation.y = current_pose.orientation.y
      pub_pose.pose.orientation.z = current_pose.orientation.z
      pub_pose.pose.orientation.w = current_pose.orientation.w
      rospy.loginfo(pub_pose)
      self.pub.publish(pub_pose)


      # time.sleep(motion_interval)


 
#robot_ip = 192.168.0.2
def main():
  try:
    print "============ Press `Enter` to start configuration ..."
    manipulator = MoveGroupPythonInteface()
 
    manipulator.robot_sensor_cali()

  

    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
 












