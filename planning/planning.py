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


class MoveGroupPythonIntefaceTutorial(object):
 
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
 
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


  def motion_loop(self):

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
 


    print "============ Press `Enter` (press ctrl-d to exit) ============"
    raw_input()

    #################################################################
    #bottom straight:
    x = -0.1
    y = 0.2
    z = 0.2
    yaw   = 0         
    pitch = -180    #capture: -180  move: -135
    roll  = 0         

    # #middle straight:
    # x = -0.1
    # y = 0.5
    # z = 0.15
    # yaw   = 0         
    # pitch = -110      #capture: -110  move: -45
    # roll  = 0         

    # #bottom curve:
    # x = -0.09
    # y = 0.3
    # z = 0.1
    # yaw   = 15         
    # pitch = -180    #capture: -180  move: -135
    # roll  = 0         

    # #cylinder:
    # x = 0
    # y = 0.5
    # z = 0.5
    # yaw   = 0         
    # pitch = -90    #capture: -90  move: -45
    # roll  = 180    #capture: 180  move: 0

    # #box:
    # x = 0
    # y = 0.5
    # z = 0.5
    # yaw   = 0         
    # pitch = -90    #capture: -90  move: -45
    # roll  = 180    #capture: 180  move: 0

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
    pub_pose.pose.orientation.x = yaw
    pub_pose.pose.orientation.y = pitch
    pub_pose.pose.orientation.z = roll
    pub_pose.pose.orientation.w = current_pose.orientation.w
    rospy.loginfo(pub_pose)
    self.pub.publish(pub_pose)
    #################################################################


    print "============ Press `Enter` (press ctrl-d to exit) ============"
    raw_input()

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

    print "============ Press `Enter` (press ctrl-d to exit) ============"
    raw_input()


    yaw = 0; pitch = -45; roll = 0; Q = euler_to_quaternion(yaw, pitch, roll)
    pose_goal = geometry_msgs.msg.Pose(); 
    pose_goal.orientation.x = Q[0]
    pose_goal.orientation.y = Q[1]
    pose_goal.orientation.z = Q[2]
    pose_goal.orientation.w = Q[3]
    pose_goal.position.x = self.motion_pathPoint[0][0] 
    pose_goal.position.y = self.motion_pathPoint[0][1] - 0.1
    pose_goal.position.z = self.motion_pathPoint[0][2] 
    group.set_pose_target(pose_goal)
    plan = group.go(joints = pose_goal, wait = True)
    group.stop()
    group.clear_pose_targets()
    print self.group.get_current_pose().pose.position
    print "yaw   : %f" % yaw
    print "pitch : %f" % pitch
    print "roll  : %f" % roll
    #################################################################

    # waypoints = []
    # wpose = geometry_msgs.msg.Pose(); i = 0
    # while i < len(self.motion_pathPoint):
    #   yaw = self.motion_pathPoint[i][3];Q = euler_to_quaternion(yaw, pitch, roll)
    #   wpose.orientation.x = Q[0]
    #   wpose.orientation.y = Q[1]
    #   wpose.orientation.z = Q[2]
    #   wpose.orientation.w = Q[3]
    #   wpose.position.x = self.motion_pathPoint[i][0] 
    #   wpose.position.y = self.motion_pathPoint[i][1]
    #   wpose.position.z = self.motion_pathPoint[i][2] 
    #   print wpose
    #   print "\n"

    #   waypoints.append(copy.deepcopy(wpose))
    #   i = i + 1

    # print waypoints
    # (plan, fraction) = group.compute_cartesian_path( waypoints, 0.01, 0 )  

    # raw_input()
    # group.execute(plan, wait=True)




    point_count = 1
    while(not rospy.is_shutdown()):

      Q = euler_to_quaternion(yaw, pitch, roll)
 
      pose_goal = geometry_msgs.msg.Pose(); 
      pose_goal.orientation.x = Q[0]
      pose_goal.orientation.y = Q[1]
      pose_goal.orientation.z = Q[2]
      pose_goal.orientation.w = Q[3]
      pose_goal.position.x = self.motion_pathPoint[point_count - 1][0] 
      pose_goal.position.y = self.motion_pathPoint[point_count - 1][1]
      pose_goal.position.z = self.motion_pathPoint[point_count - 1][2] 
      group.set_pose_target(pose_goal)
      plan = group.go(joints = pose_goal, wait = True)
      group.stop()
      group.clear_pose_targets()
      # print self.group.get_current_pose().pose.position
      # print "yaw   : %f" % yaw
      # print "pitch : %f" % pitch
      # print "roll  : %f" % roll
      # print "point_count  : %f" % point_count
      current_joints = group.get_current_joint_values()
      print current_joints[5] * 180 /pi
      print "\n"
 
      point_count = point_count + 2

      if point_count >= len(self.motion_pathPoint):
        yaw = self.motion_pathPoint[point_count - 1][3]; Q = euler_to_quaternion(yaw, pitch, roll)
        pose_goal = geometry_msgs.msg.Pose(); 
        pose_goal.orientation.x = Q[0]
        pose_goal.orientation.y = Q[1]
        pose_goal.orientation.z = Q[2]
        pose_goal.orientation.w = Q[3]
        pose_goal.position.x = self.motion_pathPoint[point_count - 1][0] 
        pose_goal.position.y = self.motion_pathPoint[point_count - 1][1] - 0.1
        pose_goal.position.z = self.motion_pathPoint[point_count - 1][2] 
        group.set_pose_target(pose_goal)
        plan = group.go(joints = pose_goal, wait = True)
        group.stop()
        group.clear_pose_targets()

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

        point_count = 1


      self.rate.sleep()

    return all_close(pose_goal, current_pose, 0.01)
 



  def linear_points_forMap(self):

    group = self.group

    x = 0.0
    y = 0.2
    z = 0.2
    yaw   = 0         #blue->z   right-hand
    pitch = -180       #red->x    right-hand    -41 is fixed angle for torch
    roll  = 0         #green->y  right-hand

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
    print current_pose.position
    print "yaw   : %f" % yaw
    print "pitch : %f" % pitch
    print "roll  : %f" % roll
    print "\n"
 

    flag = 1
    for i in range(3):
      print "============ Press `Enter` to execute a movement using a pose goal ..."
      raw_input()

      for j in range(50):
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
      pub_pose.pose.orientation.x = yaw
      pub_pose.pose.orientation.y = pitch
      pub_pose.pose.orientation.z = roll
      pub_pose.pose.orientation.w = current_pose.orientation.w
      rospy.loginfo(pub_pose)
      self.pub.publish(pub_pose)
      self.rate.sleep()

      if flag == 1:
        x = 0
        flag = 2

      elif flag == 2:
        x = -0.1
        flag = 3

    return all_close(pose_goal, current_pose, 0.01)

#robot_ip = 192.168.0.3
def main():
  try:
    # print "============ Press `Enter` to set up the moveit_commander  ..."
    # raw_input()
    ur3 = MoveGroupPythonIntefaceTutorial()
 
    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    ur3.motion_loop()

    # ur3.linear_points_forMap()
   
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
 


















  # def linear_points(self):

  #   group = self.group


  #   x = 0
  #   y = 0.35
  #   z = 0.46

  #   #axies  red->x green->y blue->z  
  #   yaw   = 0         #blue->z   right-hand
  #   pitch = -180        #red->x    right-hand
  #   roll  = 0         #green->y   right-hand

  #   flag = 1
  #   for i in range(4):
  #     print "============ Press `Enter` to execute a movement using a pose goal ..."
  #     raw_input()
  #     pose_goal = geometry_msgs.msg.Pose()
  #     Q = euler_to_quaternion(yaw , pitch + 90, roll)
  #     pose_goal.orientation.x = Q[0]
  #     pose_goal.orientation.y = Q[1]
  #     pose_goal.orientation.z = Q[2]
  #     pose_goal.orientation.w = Q[3]
  #     pose_goal.position.x = x
  #     pose_goal.position.y = y
  #     pose_goal.position.z = z
  #     group.set_pose_target(pose_goal)

  #     plan = group.go(wait=True)
  #     group.stop()

  #     group.clear_pose_targets()

  #     current_pose = self.group.get_current_pose().pose
    
  #     print current_pose.position
  #     print "yaw   : %f" % yaw
  #     print "pitch : %f" % pitch
  #     print "roll  : %f" % roll
  #     print "\n"

  #     pub_pose = PoseStamped()
  #     pub_pose.header.stamp       = rospy.Time.now()
  #     pub_pose.header.frame_id    = "robot_currentpose"
  #     pub_pose.pose.position.x    = current_pose.position.x
  #     pub_pose.pose.position.y    = current_pose.position.y
  #     pub_pose.pose.position.z    = current_pose.position.z
  #     pub_pose.pose.orientation.x = yaw
  #     pub_pose.pose.orientation.y = pitch
  #     pub_pose.pose.orientation.z = roll
  #     pub_pose.pose.orientation.w = current_pose.orientation.w
  #     rospy.loginfo(pub_pose)
  #     self.pub.publish(pub_pose)
  #     self.rate.sleep()

  #     if flag == 1:
  #       x = -0.25
  #       flag = 2

  #     elif flag == 2:
  #       x = 0
  #       flag = 3
      
  #     elif flag == 3:
  #       x = 0.25
  #       flag = 4
      
  #   return all_close(pose_goal, current_pose, 0.01)

 
  def tube_points(self):

    group = self.group


    x = 0
    y = 0.3
    z = 0.5
    yaw   = 0         #blue->z   right-hand
    pitch = -180        #red->x    right-hand
    roll  = 0         #green->y   right-hand
    
    flag = 1
    for i in range(10):
      print "============ Press `Enter` to execute a movement using a pose goal ..."
      raw_input()
      pose_goal = geometry_msgs.msg.Pose()
      Q = euler_to_quaternion(yaw , pitch + 90, roll)
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

      # time.sleep(5) 
      pub_pose = PoseStamped()
      pub_pose.header.stamp       = rospy.Time.now()
      pub_pose.header.frame_id    = "robot_currentpose"
      pub_pose.pose.position.x    = current_pose.position.x
      pub_pose.pose.position.y    = current_pose.position.y
      pub_pose.pose.position.z    = current_pose.position.z
      pub_pose.pose.orientation.x = yaw
      pub_pose.pose.orientation.y = pitch
      pub_pose.pose.orientation.z = roll
      pub_pose.pose.orientation.w = current_pose.orientation.w
      rospy.loginfo(pub_pose)
      self.pub.publish(pub_pose)
      self.rate.sleep()
      print "publish is completed!!!"
      print "\n"

      if flag == 1:
        flag = 2

        x = -0.1
        y = 0.3
        z = 0.5
        yaw   = 0         #blue->z   right-hand
        pitch = -180        #red->x    right-hand
        roll  = -10         #green->y   right-hand
    
      elif flag == 2:
        flag = 3

        x = -0.2
        y = 0.3
        z = 0.5
        yaw   = 0         #blue->z   right-hand
        pitch = -180        #red->x    right-hand
        roll  = -20         #green->y   right-hand
    
      elif flag == 3:
        flag = 4

        x = -0.3
        y = 0.3
        z = 0.4
        yaw   = 0         #blue->z   right-hand
        pitch = -180        #red->x    right-hand
        roll  = -30         #green->y   right-hand

      elif flag == 4:
        flag = 5

        x = -0.4
        y = 0.2
        z = 0.35
        yaw   = 0         #blue->z   right-hand
        pitch = -180        #red->x    right-hand
        roll  = -40         #green->y   right-hand
    
      elif flag == 5:
        flag = 6

        x = 0
        y = 0.3
        z = 0.5
        yaw   = 0         #blue->z   right-hand
        pitch = -180        #red->x    right-hand
        roll  = 0         #green->y   right-hand

      elif flag == 6:
        flag = 7

        x = 0.1
        y = 0.3
        z = 0.5
        yaw   = 0         #blue->z   right-hand
        pitch = -180        #red->x    right-hand
        roll  = 10         #green->y   right-hand
    
      elif flag == 7:
        flag = 8

        x = 0.2
        y = 0.3
        z = 0.5
        yaw   = 0         #blue->z   right-hand
        pitch = -180        #red->x    right-hand
        roll  = 20         #green->y   right-hand
    
      elif flag == 8:
        flag = 9

        x = 0.3
        y = 0.3
        z = 0.4
        yaw   = 0         #blue->z   right-hand
        pitch = -180        #red->x    right-hand
        roll  = 30         #green->y   right-hand

      elif flag == 9:
        flag = 10

        x = 0.4
        y = 0.2
        z = 0.35
        yaw   = 0         #blue->z   right-hand
        pitch = -180        #red->x    right-hand
        roll  = 40         #green->y   right-hand


      # if flag == 1:
      #   flag = 2

      #   x = 0.35
      #   y = 0.2
      #   z = 0.15
      #   yaw   = 30         #blue->z   right-hand
      #   pitch = -180        #red->x    right-hand
      #   roll  = 0         #green->y   right-hand
          

      # elif flag == 2:
      #   flag = 3

      #   x = 0
      #   y = 0.22
      #   z = 0.25
      #   yaw   = 0         #blue->z   right-hand
      #   pitch = -105        #red->x    right-hand
      #   roll  = 0         #green->y   right-hand
          
      
      # elif flag == 3:
      #   flag = 4

      #   x = -0.35
      #   y = 0.2
      #   z = 0.15
      #   yaw   = -30         #blue->z   right-hand
      #   pitch = -90        #red->x    right-hand
      #   roll  = 0         #green->y   right-hand
      
    return all_close(pose_goal, current_pose, 0.01)


