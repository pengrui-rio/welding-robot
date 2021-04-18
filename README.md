# 3D vision-based automatic welding robotic system.


## Publications:

P. Zhou, R. Peng, M. Xu, V. W. Wu and D. Navarro-Alarcon, "Path Planning with Automatic Seam Extraction over Point Cloud Models for Robotic Arc Welding," in IEEE Robotics and Automation Letters, https://ieeexplore.ieee.org/document/9394722.

R. Peng, D. Navarro-Alarcon, V. Wu and W. Yang, "A Point Cloud-Based Method for Automatic Groove Detection and Trajectory Generation of Robotic Arc Welding Tasks," 2020 17th International Conference on Ubiquitous Robots (UR), Kyoto, Japan, 2020, pp. 380-386, https://ieeexplore.ieee.org/document/9144861.


## Here are some of motion demos. 

### Tube
![image](https://github.com/professor1996/welding-robot/blob/master/demo/tube.gif)

### Cube 
![image](https://github.com/professor1996/welding-robot/blob/master/demo/cube.gif)

### Y-shape 
![image](https://github.com/professor1996/welding-robot/blob/master/demo/y-shape.gif)

### Box 
![image](https://github.com/professor1996/welding-robot/blob/master/demo/box.gif)



## Command
> roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml

> roslaunch moveit_config execution_real.launch

> roslaunch model_reconstruction  model_reconstruction.launch

> rosrun  trajectory_planning trajectory_planning_node

> rosrun traj_control_execution control_URx.py 


## Tutorial

**相机三维坐标转化为机械臂坐标需要标定相机与机械臂之间的位置关系，这个标定就称为手眼标定。**

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

### moveit的使用方法，很详细
https://myyerrol.io/zh-cn/2017/04/20/ros_experience_2_xmbot_arm/



### moveit控制ur3：
https://blog.csdn.net/qq_25267657/article/details/84871028 
 
 
1) Install the dependencies

        $ sudo apt-get install ros-kinetic-moveit
        $ sudo apt-get install ros-kinetic-universal-robot
        $ sudo apt-get install ros-kinetic-industrial-core

2) Go to catkin_ws/src


       $ git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
       $ git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git
       $ catkin_make

3) Noted that ur3_ip = 158.132.153.174


       $ ping ur3_ip to test connection

4) Launch moveit assistant to create own controller


       $ roslaunch moveit_setup_assistant setup_assistant.launch

5) Inside "moveit! assistant"
  
       5.1) Create new or edit existing?
            Choose "Create New Moveit"
            Load /home/vincent/catkin_ws/src/universal_robot/ur_description/urdf/ur3_robot.urdf.xacro
            **vincent shoudl be changed to your account name

       5.2) Optimize Self-Collision Checking
            Choose "Generate Collision Matrix"

       5.3) Define Planning Groups
            Group Name: ur3
            kinematic Solver: UR3KinematicsPlugin
            Group Default PLANNER: RRT
            Add Kin. Chain 

            "Expand All"
            Base Link = base_link
            Tip Link = tool0

       5.4) Define Robot Poses
            You can define some pose here (optional)

       5.5) Setup ROS Controllers
            Choose "Auto Add Follow Joints Trajectory ..."

       5.6) Specify Author Information
            Name: "as you like"
            Email: "as you like"

       5.7) Generate Configuration Files
            Open a new folder named "robot_moveit"
            Configre this new folder as save path
            Choose "Generate Package"

6) Create controllers.yaml


        Inside catkin_ws/src/robot_moveit/config, create a controllers.yaml
        Copy the following to it:
        //For name :(none for UR3, scaled_pos_traj_controller for UR5)
        //Never put a spacebar in the name for UR3 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

7) Create robot_planning.launch


        Inside catkin_ws/src/robot_moveit/launch
        Copy demo.launch and rename as robot_planning.launch

        ****Please comment the following if you are connected to the robot
         <!-- We do not have a robot connected, so publish fake joint states 
           <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
           <param name="use_gui" value="$(arg use_gui)"/>
           <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
         </node>  -->   

        This will give you a fake shaow which is used for simulation

        ****Change this parameter
        fake_execution -> false (for real robot)      


8) Edit robot_moveit/launch/ur3_moveit_controller_manager.launch.xml
   Replace all content by:
   
       <launch>
         <rosparam file="$(find robot_moveit)/config/controllers.yaml"/>
         <param name="use_controller_manager" value="false"/>
         <param name="trajectory_execution/execution_duration_monitoring" value="false"/>
         <param name="moveit_controller_manager" value="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
       </launch>

9) Launch and test


       $ source devel/setup.bash
       $ roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=158.132.153.174
       $ roslaunch robot_moveit robot_planning.launch


**加载自己创建的stl文件，如果出现**

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
 


 8.realsense-ros:


    roslaunch realsense2_camera rs_camera.launch 



 
 **note:**

     1.有时候arm会出现莫名其妙的旋转比如ee在执行path时突然自转一圈，这是因为ur3.urdf.xacro里限制了关节的旋转范围！！！！！！！！！
     当关节运动到临界点的时候需要越界，就会反方向旋转！！！！


     2.加上cartisian path就能使运动十分平滑


     3.result_plan = group.retime_trajectory(self.robot.get_current_state(), plan, 0.25)
       retime_trajectory函数用来调节cartisian path运动的速度


     4.在urdf文件里，link-world只用一次！！！其他所有添加的物体全部以base_link为parent link
       原因是world只是一个点，并没有指明方向，这会导致collision check失效！！！！！！！！！


     5.尤其要注意urdf文件里的collision，一定一定要和visual里的size保持一致！！
       后续添加的object link定义里，不要随便添加它的origin（xyz rpy）保持默认就好，而是尽量在joint里面修改相对位置


     6. ur5.urdf 里的joint_limited可以适当修改，否则在实际过程中机械臂可能会出现奇怪的转动

////////////////////////////////////////////////////////////////////////////////////////////////////////////


# 配置UR5与Moveit！（我用的版本是 UR5-URSoftware 3.11.0.81xxx）：

1.ip设置：

 ur5设置静态IP： 192.168.0.2
    子网掩码： 255.255.255.0
       网关： 192.168.0.10

 如果ur5是设置的静态IP，那么

 设置电脑ip：
 edit connections->IPv4 settings->add address

 IP地址不是跟机器人一样的，如机器人的IP地址是192.168.0.2
 那为了使电脑跟它处于一个局域网下只需要将最后一个小数点后面的那个设置得不一样即可，设置成 192.168.0.3 即可
 子网掩码和网关和机器人一致即可：
                       PC IP : 192.168.0.3
                      子网掩码： 255.255.255.0
                         网关： 192.168.0.10
                         
                         
2.ur5[外部控制配置](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md)：

 1.首先找到 xxx/ur_robot_driver/resources/ 下的externalcontrol-1.0.1.urcap，将这个文件拷到U盘里，然后再拷到UR5的teach-pendant上

 2.开启UR5，点击setup robot -> URCaps-> "+" -> usbdisk -> externalcontrol-1.0.1.urcap -> Restart
 
 3.重新开启后，点击program robot -> Installation -> External Control -> Setup PC's IP (192.168.0.3)

 4.点击program robot -> Program -> Empty Program -> Structure -> URCaps -> External Control -> Save the file (roscontrol)

 5.回到主菜单点击Run Program -> File -> Load Program -> 选择之前保存的urp文件: roscontrol.urp -> 底部有个开始的button, 但是要等roslaunch ur5_bringup.launch ..... 之后才能点
 
 6.看到右边Variables 里面出现了 机器人关节信息就说明已经setup 完毕了， 现在可以开始控制UR5了
 
 
3.[Universal_Robots_ROS_Driver配置](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

 1. source global ros:
   $ source /opt/ros/<your_ros_version>/setup.bash

   create a catkin workspace:
   $ mkdir -p catkin_ws/src && cd catkin_ws

   clone the driver:
   $ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

   clone fork of the description to use the calibration feature:
   $ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

   install dependencies:
   $ sudo apt update -qq
   $ rosdep update
   $ rosdep install --from-path src --ignore-src -y  （注意要在工作空间的路径下，而不是src路径下！！！，这个命令会安装很多控制器）

   build the workspace:
   $ catkin_make

   activate the workspace (ie: source it):
   $ source devel/setup.bash
   
   
  2.标定UR5：
  
   roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.0.2 target_filename:="${HOME}/my_robot_calibration.yaml"
 
   创建标定文件  .yaml
   
   
  3.开启UR5 Driver：
  
   roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2
   
   或者加入标定文件：
   
   roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.0.2  kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml 

   (开启成功要注意的点是： 保证ur_description文件夹下面有/ur_description/launch/ur5_upload.launch 这个xx_upload.launch文件， 其他型号也一样)
   
   开启这个launch文件后再回到UR5的控制面板上点击上面说的开始按钮，在run program的最下面， 然后终端里最下面会有出现：
   
   [ INFO] [1582949896.667813900]: Robot requested program
   [ INFO] [1582949896.667942812]: Sent program to robot
   [ INFO] [1582949896.711829650]: Robot ready to receive control commands.    这就说明UR5已经和PC driver建立联系了，可以开始PC控制了


4.配置UR5的Moveit！
  
  1.首先用assistant setup软件配置UR5的工作环境， 相关urdf都准备好，设置好控制组
  
  2.在创建的moveit包下面的config文件夹里面，新创建一个controllers.yaml:
  
   controller_list:
      - name: "scaled_pos_traj_controller"
        action_ns: follow_joint_trajectory
        type: FollowJointTrajectory
        joints:
          - shoulder_pan_joint
          - shoulder_lift_joint
          - elbow_joint
          - wrist_1_joint
          - wrist_2_joint
          - wrist_3_joint
          
  3.创建Moveit相关的launch文件：
  
   1.创建robot_moveit/launch/robot_planning.launch（copy from demo.launch）:

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
 
  2.修改robot_moveit/launch/xxx_moveit_controller_manager.launch.xml :

           <launch>
            <rosparam file="$(find robot_moveit)/config/controllers.yaml"/>
            <param name="use_controller_manager" value="false"/>
            <param name="trajectory_execution/execution_duration_monitoring" value="false"/>
            <param name="moveit_controller_manager" value="moveit_simple_controller_manager/MoveItSimpleControllerManager"/>
           </launch>

   最后roslaunch robot_moveit robot_planning.launch就可以看到rViz里面的虚拟环境了
   可以用Motion Planning 模块 手动测试机械臂运动状态
   也可以用Planning Scene 通过代码来控制


