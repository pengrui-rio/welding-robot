# welding-robot  

roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml

roslaunch moveit_config execution_real.launch

rosrun control control.py

roslaunch realsense2_camera rs_camera.launch align_depth:=true

roslaunch ar_track_alvar ar_indiv.launch

rosrun weldingrobot_sensor_calibration weldingrobot_sensor_calibration_node

rosrun trajectory_planning trajectory_planning_node

相机三维坐标转化为机械臂坐标需要标定相机与机械臂之间的位置关系，这个标定就称为手眼标定。

video link: : https://vimeo.com/371773986
 
需要git clone 默认package

/////////////////////////////

cd src

git clone https://github.com/IntelRealSense/realsense-ros.git

cd realsense-ros/  然后delete realsense2_description文件夹

git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`


cd src


git clone https://github.com/pal-robotics/ddynamic_reconfigure.git


git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git


///////////////////////////////////////////////////////////////////////////////////////

moveit的使用方法，很详细
https://myyerrol.io/zh-cn/2017/04/20/ros_experience_2_xmbot_arm/



moveit控制ur3：
https://blog.csdn.net/qq_25267657/article/details/84871028 
 
 
1.
sudo apt-get install ros-kinetic-moveit

sudo apt-get install ros-kinetic-universal-robot
 
sudo apt-get install ros-kinetic-industrial-core

2.
cd ws/src/
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git

cd universal_robot/
delete ur_driver

git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git

cd ws
catkin_make

3.
如果ur是设置的静态IP，那么

设置电脑ip：
edit connections->IPv4 settings->add address
 
IP地址不是跟机器人一样的，如机器人的IP地址是192.168.1.10，那为了使电脑跟它处于一个局域网下
只需要将最后一个小数点后面的那个设置得不一样即可，设置成 192.168.1.11 即可
子网掩码和网关和机器人一致即可


如果ur是设置的DHCP，那么

设置电脑ip：
edit connections->IPv4 settings->DHCP即可
ur会自动给电脑分配一个IP， 通过ping ur自己的IP  测试连接状态


4.
moveit控制ur3  这是官方包的控制方法
roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=158.132.153.253
roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch 
roslaunch ur3_moveit_config moveit_rviz.launch config:=true


5.roslaunch moveit_setup_assistant setup_assistant.launch  这是自己配置仿真环境和控制平台

  选择robot.xacro文件并加载, 但不要修改ur的最底层源xacro文件！！！直接加载就行，否则后续pose和joint控制会出问题！～！
  
  generate collision matrix
  
  planning group: select chain from base-link to wrist3_link (或者end-effector)   命名 "ur3"！
            ！！(to tool/其他物体 -> end-effector就为tool/其他物体, 在urdf里设置tool和其他物体之间的joint位置就可以设置transformation的位置)！！
                                                
  end effector group: select end-effector link  and add
  
  check robot pose
  
  add end effectors
  ##auto generate ROS controllers
  
  email+name
  
  build one folder in your workspace  一般取名叫robot_moveit
  
  save and generate
  
  create planning package
  
  
 6. 
 robot_moveit/config  创建controllers.yaml:

 controller_list:
  - name: ""
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
 
 创建robot_moveit/launch/robot_planning.launch（copy from demo.launch）:
     
     主要内容：
     
     1.planning_context.launch 
     Load the URDF, SRDF and other .yaml configuration files on the param server
       
     2.move_group.launch
     Run the main MoveIt! executable without trajectory execution
     
     3.moveit_rviz.launch
     Run Rviz and load the default config to see the state of the move_group node
     
     4.default_warehouse_db.launch
     Load database
     
     
       Tips：
       
       1.控制实物时一定要注释这个node，否则会发fake pose干扰motion planning
       <!-- We do not have a robot connected, so publish fake joint states 
       <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
         <param name="use_gui" value="$(arg use_gui)"/>
         <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
       </node>  --> 

       2.修改参数
       fake_execution->false   控制实物

                       true    仿真模拟
 
 修改robot_moveit/launch/xxx_moveit_controller_manager.launch.xml :
 
       <launch>
        <rosparam file="$(find robot_moveit)/config/controllers.yaml"/>
        <param name="use_controller_manager" value="false"/>
        <param name="trajectory_execution/execution_duration_monitoring" value="false"/>
        <param name="moveit_controller_manager" value="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
       </launch>
            
 执行： 
 roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=ur自己的IP
 
 roslaunch robot_moveit robot_planning.launch
 
 
      加载自己创建的stl文件，如果出现 
      It starts with the word 'solid', indicating that it's an ASCII STL file, 
      but it does not contain the word 'endsolid' soit is either a malformed ASCII STL file or it is actually a binary STL file. 
      Trying to interpret it as a binary STL file instead.

      则进入该stl文件目录下开启一个终端，输入：
      sed -i 's/^solid/robot/' *


 
 7.创建planning/planning.py   这里写你的控制命令
 
 chmod +x planning.py
 
 rosrun planning planning.py
 
 (所有的pose都是相对于Reference frame: /world的!!!!!!!!!!!!!!! 注意初始化命令行给出的Reference frame！！！！)
 (所以可以把world frame 和 base-link frame合并，在urdf文件中标好位置)
 
 
 
 note:
 1.有时候arm会出现莫名其妙的旋转比如ee在执行path时突然自转一圈，这是因为ur3.urdf.xacro里限制了关节的旋转范围！！！！！！！！！
 当关节运动到临界点的时候需要越界，就会反方向旋转！！！！
 
 
 2.加上cartisian path就能使运动十分平滑



 8.realsense-ros:
 roslaunch realsense2_camera rs_camera.launch 
