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

    rospy.Subscriber("motion_Path", Pose, self.callback_path)
    pub = rospy.Publisher('robot_currentpose', PoseStamped, queue_size=10)
    rate = rospy.Rate(1000) # 1000hz


    robot = moveit_commander.RobotCommander()
 
    scene = moveit_commander.PlanningSceneInterface()
 
    group_name = "ur3"
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


  def robot_sensor_cali(self):
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

    #################################################################
    #对应哪个颜色的柱子，右手坐标系，转多少度，顺序按yaw->pitch->roll,也有很大可能任意顺序
    #yaw->purple   pitch->red   roll->green

    #bottom straight:
    x = 0.0
    y = 0.25
    z = 0.45 

    # x = 0.0
    # y = 0.44
    # z = 0.25            
           
    yaw   = 0       
    pitch = 180               
    roll  =  0  
     


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

    plan = group.go(wait=True)
    group.stop()
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



  def motion(self):
    print "============ Press `Enter` to set the initial pose ..."
    raw_input()

    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/200
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = 0
 
    group.go(joint_goal, wait=True)
    group.stop()
    group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose

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
 

    # print "============ Press `Enter` to capture pointcloud ============"
    # raw_input()
    
    # #bottom straight:
    # x = 0
    # y = 0.3
    # z = 0.4
    # yaw   = 179.589         
    # pitch = -0.859868    #capture: -180  move: -135
    # roll  = 179.339         

    # pose_goal = geometry_msgs.msg.Pose()
    # Q = euler_to_quaternion(yaw , pitch, roll)
    # pose_goal.orientation.x = Q[0]
    # pose_goal.orientation.y = Q[1]
    # pose_goal.orientation.z = Q[2]
    # pose_goal.orientation.w = Q[3]
    # pose_goal.position.x = x
    # pose_goal.position.y = y
    # pose_goal.position.z = z
    # group.set_pose_target(pose_goal)

    # plan = group.go(wait=True)
    # group.stop()
    # group.clear_pose_targets()
    # current_pose = self.group.get_current_pose().pose
    # print current_pose.position
    # print "yaw   : %f" % yaw
    # print "pitch : %f" % pitch
    # print "roll  : %f" % roll
    # print "\n"

    # pub_pose = PoseStamped()
    # pub_pose.header.stamp       = rospy.Time.now()
    # pub_pose.header.frame_id    = "robot_currentpose"
    # pub_pose.pose.position.x    = current_pose.position.x
    # pub_pose.pose.position.y    = current_pose.position.y
    # pub_pose.pose.position.z    = current_pose.position.z
    # pub_pose.pose.orientation.x = yaw
    # pub_pose.pose.orientation.y = pitch
    # pub_pose.pose.orientation.z = roll
    # pub_pose.pose.orientation.w = 0
    # rospy.loginfo(pub_pose)
    # self.pub.publish(pub_pose)
    # #################################################################


    # print "============ Press `Enter` to sent signal for processing the pointcloud ============"
    # raw_input()
    # pub_pose = PoseStamped()
    # pub_pose.header.stamp       = rospy.Time.now()
    # pub_pose.header.frame_id    = "robot_currentpose"
    # pub_pose.pose.position.x    = current_pose.position.x
    # pub_pose.pose.position.y    = current_pose.position.y
    # pub_pose.pose.position.z    = current_pose.position.z
    # pub_pose.pose.orientation.x = yaw
    # pub_pose.pose.orientation.y = pitch
    # pub_pose.pose.orientation.z = roll
    # pub_pose.pose.orientation.w = 1
    # rospy.loginfo(pub_pose)
    # self.pub.publish(pub_pose)


    #################################################################

    print "============ Press `Enter` to start execution ============"
    raw_input()
    offset_x = 0
    offset_y = -0.01
    offset_z = 0.02
    p = []; p.append(-0.182909); p.append(0.469571 + offset_y); p.append(0.022352 + offset_z); self.motion_pathPoint.append( p )
    p = []; p.append(-0.147072); p.append(0.442188 + offset_y); p.append(0.017352 + offset_z); self.motion_pathPoint.append( p )
    p = []; p.append(-0.103719); p.append(0.419921 + offset_y); p.append(0.018352 + offset_z); self.motion_pathPoint.append( p )
    p = []; p.append(-0.0500075); p.append(0.406866 + offset_y); p.append(0.019352 + offset_z); self.motion_pathPoint.append( p )
    p = []; p.append(0.0124542); p.append(0.421396 + offset_y); p.append(0.024352 + offset_z); self.motion_pathPoint.append( p )
    p = []; p.append(0.0792019); p.append(0.452637 + offset_y); p.append(0.021352 + offset_z); self.motion_pathPoint.append( p )
    p = []; p.append(0.144905); p.append(0.469668 + offset_y); p.append(0.019352 + offset_z); self.motion_pathPoint.append( p )
    p = []; p.append(0.181533); p.append(0.455276 + offset_y); p.append(0.019352 + offset_z); self.motion_pathPoint.append( p )
    p = []; p.append(0.217224); p.append(0.407183 + offset_y); p.append(0.026352 + offset_z); self.motion_pathPoint.append( p )
    print self.motion_pathPoint


    yaw = 0; pitch = -135; roll = 0; Q = euler_to_quaternion(yaw, pitch, roll)
    pose_goal = geometry_msgs.msg.Pose(); 
    pose_goal.orientation.x = Q[0]
    pose_goal.orientation.y = Q[1]
    pose_goal.orientation.z = Q[2]
    pose_goal.orientation.w = Q[3]
    pose_goal.position.x = self.motion_pathPoint[0][0] 
    pose_goal.position.y = self.motion_pathPoint[0][1] 
    pose_goal.position.z = self.motion_pathPoint[0][2] + 0.1
    group.set_pose_target(pose_goal)
    plan = group.go(joints = pose_goal, wait = True)
    group.stop()
    group.clear_pose_targets()
    print self.group.get_current_pose().pose.position
    print "yaw   : %f" % yaw
    print "pitch : %f" % pitch
    print "roll  : %f" % roll
 

    waypoints = []
    wpose = geometry_msgs.msg.Pose(); i = 0
    while i < len(self.motion_pathPoint):
      yaw = 0; pitch = -135; roll = 0; Q = euler_to_quaternion(yaw, pitch, roll)
      wpose.orientation.x = Q[0]
      wpose.orientation.y = Q[1]
      wpose.orientation.z = Q[2]
      wpose.orientation.w = Q[3]
      wpose.position.x = self.motion_pathPoint[i][0] 
      wpose.position.y = self.motion_pathPoint[i][1]
      wpose.position.z = self.motion_pathPoint[i][2] 
      print wpose
      print "\n"

      waypoints.append(copy.deepcopy(wpose))
      i = i + 1

    print waypoints
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0)   
    raw_input()

    result_plan = group.retime_trajectory(self.robot.get_current_state(), plan, 0.05)
    group.execute(result_plan, wait=True)

    #################################################################
    print "============ Back to initial status ============"
    result_plan = group.retime_trajectory(self.robot.get_current_state(), plan, 1)
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

    return all_close(pose_goal, current_pose, 0.01)


 
 


#robot_ip = 192.168.0.2
def main():
  try:
    print "============ Press `Enter` to start configuration ..."
    ur3 = MoveGroupPythonInteface()
 
    ur3.robot_sensor_cali()

    # ur3.motion()
 

    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
 












