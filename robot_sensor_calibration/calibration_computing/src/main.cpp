#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>
#include <algorithm.h>
#include <transformation.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Bool.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

// PCL lib
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <transformation.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>//点云文件pcd 读写
#include <pcl/visualization/cloud_viewer.h>//点云可视化
#include <pcl/visualization/pcl_visualizer.h>// 高级可视化点云类
#include <pcl/features/normal_3d.h>//法线特征
#include <pcl/kdtree/kdtree_flann.h>//搜索方法
#include <boost/thread/thread.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
 
// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud; 
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloudL;  
typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::Normal> Normal;

using namespace cv;
using namespace std;
using namespace Eigen;
//namespace enc = sensor_msgs::image_encodings;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 320;
const double camera_cy = 240;
const double camera_fx = 619.966;
const double camera_fy = 619.856;

// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;

//receive robot current pose flag
int receive_pose_flag = 0, process_flag = 0;
float current_x = 0  , current_y = 0    , current_z = 0;
float current_yaw = 0, current_pitch = 0, current_roll = 0;
float current_orientation_x = 0, current_orientation_y = 0, current_orientation_z = 0, current_orientation_w = 0;  

//receive marker current pose
float marker_center_x = 0, marker_center_y = 0, marker_center_z = 0;

void marker_info_Callback(const geometry_msgs::Pose& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{

  marker_center_x     = msg.position.x;
  marker_center_y     = msg.position.y; 
  marker_center_z     = msg.position.z;

  // cout << "_x: " << marker_center_x << " _y: " << marker_center_y << " _z: " << marker_center_z << endl;
}

void robot_currentpose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{
  // ROS_INFO("I heard the pose from the robot"); 
  // ROS_INFO("the position(x,y,z) is %f , %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  // ROS_INFO("the orientation(yaw, pitch, roll) is %f , %f, %f ", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  // ROS_INFO("the time we get the pose is %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec);

  current_x     = msg->pose.position.x;
  current_y     = msg->pose.position.y; 
  current_z     = msg->pose.position.z;

  current_orientation_x  = msg->pose.orientation.x;
  current_orientation_y  = msg->pose.orientation.y;
  current_orientation_z  = msg->pose.orientation.z;
  current_orientation_w  = msg->pose.orientation.w;

  // cout << endl << endl;
}
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
void EyeinHand_compute_TEC(geometry_msgs::Pose Base_End);
Point3f EyeinHand_Result_validation(geometry_msgs::Pose Base_End);
void EyetoHand_compute_TBC(void);


int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "robot_sensor_calibration");
  ros::NodeHandle nh;
  ros::Rate naptime(10); // use to regulate loop rate 
 
  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber robot_currentpose_sub = nh.subscribe("robot_currentpose", 10, robot_currentpose_Callback);
  ros::Subscriber marker_info_sub       = nh.subscribe("marker_info"      , 10, marker_info_Callback);

  // EyetoHand_compute_TBC();
  // EyeinHand_compute_TEC();
  // EyeinHand_Result_validation();
  cout << " \n" << endl; 


  tf::TransformListener listener;
  tf::TransformBroadcaster tf_broadcaster;

  int i = 0;
  while (ros::ok()) 
  { 
    // listen other coordinates
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/base_link", "/tool0", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::Pose Base_End;
    Base_End.position.x = transform.getOrigin().x();      Base_End.position.y = transform.getOrigin().y();      Base_End.position.z = transform.getOrigin().z();
    Base_End.orientation.x = transform.getRotation().x(); Base_End.orientation.y = transform.getRotation().y(); Base_End.orientation.z = transform.getRotation().z(); Base_End.orientation.w = transform.getRotation().w();
    // cout << "Base_End: " << endl << Base_End << endl;

    i = i + 1;
    if ((i % 100000) == 0)
    {
      i = 0;
      EyeinHand_compute_TEC(Base_End);
    }


    ros::spinOnce(); //allow data update from callback; 
    // naptime.sleep(); // wait for remainder of specified period; 
  }


  ros::spin();
 
}


void EyetoHand_compute_TBC(void)
{
  // Camera -> object
	Matrix4d T_C_o; 
  float T_C_o_Q[4];
  euler_to_quaternion(0, 180, 0, T_C_o_Q);
  float T_C_o_r[9];
  Quaternion_to_RotationMatrix(T_C_o_Q[0], T_C_o_Q[1], T_C_o_Q[2], T_C_o_Q[3], T_C_o_r);//camera_color_optical_frame
  // euler_to_RotationMatrix(-90, -180, 0, T_C_o_r);
	T_C_o << T_C_o_r[0], T_C_o_r[1], T_C_o_r[2], 0.0389463,
           T_C_o_r[3], T_C_o_r[4], T_C_o_r[5], 0.00319754,
           T_C_o_r[6], T_C_o_r[7], T_C_o_r[8], 0.578,
           0,          0,          0,          1;
  cout << "T_C_o: " << endl << T_C_o << endl;
  // cout << "purple: " << 180*atan2(T_C_o_r[2], T_C_o_r[8]) / M_PI << endl;
  // cout << "red: "    << 180*atan2(T_C_o_r[3], T_C_o_r[4]) / M_PI << endl;
  // cout << "green: "  << 180*asin(-T_C_o_r[5])             / M_PI << endl;
  cout << endl;


  // Base -> object
	Matrix4d T_B_o; 
  float T_B_o_Q[4];
  euler_to_quaternion(0, 0, 0, T_B_o_Q);
  float T_B_o_r[9];
  Quaternion_to_RotationMatrix(T_B_o_Q[0], T_B_o_Q[1], T_B_o_Q[2], T_B_o_Q[3], T_B_o_r);
  // euler_to_RotationMatrix(90, 0, 0, T_o_B_r);
	T_B_o << T_B_o_r[0], T_B_o_r[1], T_B_o_r[2], -0.0016,
           T_B_o_r[3], T_B_o_r[4], T_B_o_r[5], 0.440399316218,
           T_B_o_r[6], T_B_o_r[7], T_B_o_r[8], 0.252352433451,
           0,          0,          0,          1;
  cout << "T_B_o: " << endl << T_B_o << endl;
  // cout << "purple: " << 180*atan2(T_o_B_r[2], T_o_B_r[8]) / M_PI << endl;
  // cout << "red: "    << 180*atan2(T_o_B_r[3], T_o_B_r[4]) / M_PI << endl;
  // cout << "green: "  << 180*asin(-T_o_B_r[5])             / M_PI << endl;
  cout << endl;



  // Base -> Camera
	Matrix4d T_B_C; 

	T_B_C = T_B_o * T_C_o.inverse();
	cout << "T_B_C: " << endl << T_B_C << endl;
  // cout << "purple: " << 180*atan2(T_E_C(0,2), T_E_C(2,2)) / M_PI << endl;
  // cout << "red: "    << 180*atan2(T_E_C(1,0), T_E_C(1,1)) / M_PI << endl;
  // cout << "green: "  << 180*asin(-T_E_C(1,2))             / M_PI << endl;

}


void EyetoHand_Result_validation(Point3f path_point_3D)
{
  // Base -> Camera
	Matrix4d T_B_C; 

  T_B_C << 1,            0,           -0,   -0.0405463,
           0,           -1, -8.74228e-08,     0.443597,
           0,  8.74228e-08,           -1,     0.830352,
           0,            0,            0,            1;

  // Camera -> object
	Matrix4d T_C_o; 
  float T_C_o_Q[4];
  euler_to_quaternion(0, 180, 0, T_C_o_Q);
  float T_C_o_r[9];
  Quaternion_to_RotationMatrix(T_C_o_Q[0], T_C_o_Q[1], T_C_o_Q[2], T_C_o_Q[3], T_C_o_r);//camera_color_optical_frame
  // euler_to_RotationMatrix(-90, -180, 0, T_C_o_r);
	T_C_o << T_C_o_r[0], T_C_o_r[1], T_C_o_r[2], 0.0389463,
           T_C_o_r[3], T_C_o_r[4], T_C_o_r[5], 0.00319754,
           T_C_o_r[6], T_C_o_r[7], T_C_o_r[8], 0.578,
           0,          0,          0,          1;
  cout << "T_C_o: " << endl << T_C_o << endl;


  // Base -> object
	Matrix4d T_B_o; 

  T_B_o = T_B_C * T_C_o;
  cout << "T_B_o: " << endl << T_B_o << endl;
  cout << endl;

}


void EyeinHand_compute_TEC(geometry_msgs::Pose Base_End)
{
  //Base -> End:
  float T_B_E_Q[4];
  // euler_to_quaternion(20, -170, 0, T_B_E_Q);
  T_B_E_Q[0] = Base_End.orientation.x;
  T_B_E_Q[1] = Base_End.orientation.y;
  T_B_E_Q[2] = Base_End.orientation.z;
  T_B_E_Q[3] = Base_End.orientation.w;

  float T_B_E_r[9];
  Quaternion_to_RotationMatrix(T_B_E_Q[0], T_B_E_Q[1], T_B_E_Q[2], T_B_E_Q[3], T_B_E_r);
  // euler_to_RotationMatrix(0, -180, 0, T_B_E_r);
  Matrix4d T_B_E; // Base -> End
	T_B_E << T_B_E_r[0], T_B_E_r[1], T_B_E_r[2], Base_End.position.x,
           T_B_E_r[3], T_B_E_r[4], T_B_E_r[5], Base_End.position.y,
           T_B_E_r[6], T_B_E_r[7], T_B_E_r[8], Base_End.position.z,
           0,          0,          0,          1;
 

  // Camera -> object
	Matrix4d T_C_o; 
  float T_C_o_Q[4];
  euler_to_quaternion(0, 0, 0, T_C_o_Q);
  float T_C_o_r[9];
  Quaternion_to_RotationMatrix(T_C_o_Q[0], T_C_o_Q[1], T_C_o_Q[2], T_C_o_Q[3], T_C_o_r);//camera_color_optical_frame
  // euler_to_RotationMatrix(-90, -180, 0, T_C_o_r);
	T_C_o << T_C_o_r[0], T_C_o_r[1], T_C_o_r[2], marker_center_x,
           T_C_o_r[3], T_C_o_r[4], T_C_o_r[5], marker_center_y,
           T_C_o_r[6], T_C_o_r[7], T_C_o_r[8], marker_center_z,
           0,          0,          0,          1;
 

  // object -> Base
	Matrix4d T_B_o; 
  float T_B_o_Q[4];
  // euler_to_quaternion(90, 0, 0, T_B_o_Q);
  T_B_o_Q[0] = Base_End.orientation.x;
  T_B_o_Q[1] = Base_End.orientation.y;
  T_B_o_Q[2] = Base_End.orientation.z;
  T_B_o_Q[3] = Base_End.orientation.w;


  float T_B_o_r[9];
  Quaternion_to_RotationMatrix(T_B_o_Q[0], T_B_o_Q[1], T_B_o_Q[2], T_B_o_Q[3], T_B_o_r);
  // euler_to_RotationMatrix(90, 0, 0, T_o_B_r);
	T_B_o << T_B_o_r[0], T_B_o_r[1], T_B_o_r[2], 0,
           T_B_o_r[3], T_B_o_r[4], T_B_o_r[5], 0.524,
           T_B_o_r[6], T_B_o_r[7], T_B_o_r[8], 0.0001,
           0,          0,          0,          1;


  // End -> Camera
	Matrix4d T_E_C; 

	T_E_C = T_B_E.inverse() * T_B_o * T_C_o.inverse();
	cout << "T_E_C: " << endl << T_E_C << endl;

  // T_E_C: 
  //            1   2.7798e-18            0   -0.0401405
  // -5.57321e-19            1  2.71051e-20    -0.116625
  //   1.0842e-19            0            1    0.0707586
  //            0            0            0            1

}

Point3f EyeinHand_Result_validation(geometry_msgs::Pose Base_End)
{
  //Base -> End:
  float T_B_E_Q[4];
  T_B_E_Q[0] = Base_End.orientation.x;
  T_B_E_Q[1] = Base_End.orientation.y;
  T_B_E_Q[2] = Base_End.orientation.z;
  T_B_E_Q[3] = Base_End.orientation.w;

  float T_B_E_r[9];
  Quaternion_to_RotationMatrix(T_B_E_Q[0], T_B_E_Q[1], T_B_E_Q[2], T_B_E_Q[3], T_B_E_r);
  // euler_to_RotationMatrix(0, -180, 0, T_B_E_r);
  Matrix4d T_B_E; // Base -> End
	T_B_E << T_B_E_r[0], T_B_E_r[1], T_B_E_r[2], Base_End.position.x,
           T_B_E_r[3], T_B_E_r[4], T_B_E_r[5], Base_End.position.y,
           T_B_E_r[6], T_B_E_r[7], T_B_E_r[8], Base_End.position.z,
           0,          0,          0,          1;



  // End -> Camera
  Matrix4d T_E_C;
  T_E_C <<   1,   2.7798e-18,            0,   -0.0387791,
  -5.57321e-19,            1,  2.71051e-20,    -0.116142,
    1.0842e-19,            0,            1,    0.0708987,
             0,            0,            0,            1;
  // cout << "T_E_C: " << endl << T_E_C << endl;


  // Base -> Camera
  Matrix4d T_B_C;
  T_B_C = T_B_E * T_E_C;
  // cout << "T_B_C: " << endl << T_B_C << endl;


  // Camera -> object  0.0691944 -0.0164191
	Matrix4d T_C_o; 
  float T_C_o_Q[4];
  euler_to_quaternion(0, 0, 0, T_C_o_Q);
  float T_C_o_r[9];
  Quaternion_to_RotationMatrix(T_C_o_Q[0], T_C_o_Q[1], T_C_o_Q[2], T_C_o_Q[3], T_C_o_r);//camera_color_optical_frame
  // euler_to_RotationMatrix(-90, -180, 0, T_C_o_r);
	T_C_o << T_C_o_r[0], T_C_o_r[1], T_C_o_r[2], marker_center_x,
           T_C_o_r[3], T_C_o_r[4], T_C_o_r[5], marker_center_y,
           T_C_o_r[6], T_C_o_r[7], T_C_o_r[8], marker_center_z,
           0,          0,          0,          1;
  // cout << "T_C_o: " << endl << T_C_o << endl;

  // result by calibration:
  Matrix4d T_R;
  T_R = T_B_C * T_C_o;
  // cout << "T_R: " << endl << T_R << endl;

  Point3f object_position_world;
  object_position_world.x = T_R(0,3);
  object_position_world.y = T_R(1,3);
  object_position_world.z = T_R(2,3);

  return  object_position_world;
}

