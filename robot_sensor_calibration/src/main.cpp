#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>
#include <algorithm.h>
#include <transformation.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
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


void Quaternion_to_RotationMatrix(float x, float y, float z, float w, float R[9]);
void euler_to_quaternion(float Yaw, float Pitch, float Roll, float Q[4]);
void euler_to_RotationMatrix(float Yaw, float Pitch, float Roll, float R[9]);

void robot_currentpose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{

  ROS_INFO("I heard the pose from the robot"); 
  ROS_INFO("the position(x,y,z) is %f , %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  ROS_INFO("the orientation(yaw, pitch, roll) is %f , %f, %f ", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  ROS_INFO("the time we get the pose is %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec);

  current_x     = msg->pose.position.x;
  current_y     = msg->pose.position.y; 
  current_z     = msg->pose.position.z;

  current_orientation_x  = msg->pose.orientation.x;
  current_orientation_y  = msg->pose.orientation.y;
  current_orientation_z  = msg->pose.orientation.z;
  current_orientation_w  = msg->pose.orientation.w;

  cout << endl << endl;


}
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////
 

int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "robot_sensor_calibration");
  ros::NodeHandle nh;
  ros::Rate naptime(10); // use to regulate loop rate 
 
  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("robot_currentpose", 10, robot_currentpose_Callback);


  //Base -> End:
  float T_B_E_Q[4];
  euler_to_quaternion(20, -170, 0, T_B_E_Q);
  T_B_E_Q[0] = -0.999951264231;
  T_B_E_Q[1] = -0.00534872895094;
  T_B_E_Q[2] = -0.00359072703145;
  T_B_E_Q[3] = 0.0074811055935;

  float T_B_E_r[9];
  Quaternion_to_RotationMatrix(T_B_E_Q[0], T_B_E_Q[1], T_B_E_Q[2], T_B_E_Q[3], T_B_E_r);
  // euler_to_RotationMatrix(0, -180, 0, T_B_E_r);
  Matrix4d T_B_E; // Base -> End
	T_B_E << T_B_E_r[0], T_B_E_r[1], T_B_E_r[2], 0.0187,
           T_B_E_r[3], T_B_E_r[4], T_B_E_r[5], 0.2508,
           T_B_E_r[6], T_B_E_r[7], T_B_E_r[8], 0.4520,
           0,          0,          0,          1;
  cout << "T_B_E: " << endl << T_B_E << endl;
  cout << "purple: " << 180*atan2(T_B_E_r[2], T_B_E_r[8]) / M_PI << endl;
  cout << "red: "    << 180*atan2(T_B_E_r[3], T_B_E_r[4]) / M_PI << endl;
  cout << "green: "  << 180*asin(-T_B_E_r[5])             / M_PI << endl;
  cout << endl;


  // Camera -> object
	Matrix4d T_C_o; 
  float T_C_o_Q[4];
  euler_to_quaternion(-90, -180, 0, T_C_o_Q);
  float T_C_o_r[9];
  Quaternion_to_RotationMatrix(T_C_o_Q[0], T_C_o_Q[1], T_C_o_Q[2], T_C_o_Q[3], T_C_o_r);//camera_color_optical_frame
  // euler_to_RotationMatrix(-90, -180, 0, T_C_o_r);
	T_C_o << T_C_o_r[0], T_C_o_r[1], T_C_o_r[2], -0.0326393,
           T_C_o_r[3], T_C_o_r[4], T_C_o_r[5], -0.0169698,
           T_C_o_r[6], T_C_o_r[7], T_C_o_r[8], 0.4194,
           0,          0,          0,          1;
  cout << "T_C_o: " << endl << T_C_o << endl;
  cout << "purple: " << 180*atan2(T_C_o_r[2], T_C_o_r[8]) / M_PI << endl;
  cout << "red: "    << 180*atan2(T_C_o_r[3], T_C_o_r[4]) / M_PI << endl;
  cout << "green: "  << 180*asin(-T_C_o_r[5])             / M_PI << endl;
  cout << endl;


  // object -> Base
	Matrix4d T_o_B; 
  float T_o_B_Q[4];
  euler_to_quaternion(90, 0, 0, T_o_B_Q);
  float T_o_B_r[9];
  Quaternion_to_RotationMatrix(T_o_B_Q[0], T_o_B_Q[1], T_o_B_Q[2], T_o_B_Q[3], T_o_B_r);
  // euler_to_RotationMatrix(90, 0, 0, T_o_B_r);
	T_o_B << T_o_B_r[0], T_o_B_r[1], T_o_B_r[2], -0.04,
           T_o_B_r[3], T_o_B_r[4], T_o_B_r[5], 0.328,
           T_o_B_r[6], T_o_B_r[7], T_o_B_r[8], 0.001,
           0,          0,          0,          1;
  cout << "T_o_B: " << endl << T_o_B << endl;
  cout << "purple: " << 180*atan2(T_o_B_r[2], T_o_B_r[8]) / M_PI << endl;
  cout << "red: "    << 180*atan2(T_o_B_r[3], T_o_B_r[4]) / M_PI << endl;
  cout << "green: "  << 180*asin(-T_o_B_r[5])             / M_PI << endl;
  cout << endl;



  // End -> Camera
	Matrix4d T_E_C; 

	T_E_C = T_B_E.inverse() * T_o_B * T_C_o.inverse();
	cout << "T_E_C: " << endl << T_E_C << endl;


  cout << " \n" << endl; //add two more blank row so that we can see the message more clearly


  tf::TransformListener listener;

  while (ros::ok()) 
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/base_link", "/ar_marker_1", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // cout << "CamToOutput.getOrigin: " << transform.getOrigin().x() << " "
    //                                   << transform.getOrigin().y() << " " 
    //                                   << transform.getOrigin().z()<< endl;
    // cout << "CamToOutput.getrotation: " << transform.getRotation().x() << " " 
    //                                     << transform.getRotation().y() << " " 
    //                                     << transform.getRotation().z() << " " 
    //                                     << transform.getRotation().w() << endl<< endl;

    ros::spinOnce(); //allow data update from callback; 
    // naptime.sleep(); // wait for remainder of specified period; 
  }


  ros::spin();
 
}

//四元数 -> 旋转矩阵
void Quaternion_to_RotationMatrix(float x, float y, float z, float w, float R[9])
{
  float r11 = 0, r12 = 0, r13 = 0;
  float r21 = 0, r22 = 0, r23 = 0; 
  float r31 = 0, r32 = 0, r33 = 0;

  R[0]  = 1 - 2 * y * y - 2 * z * z;
  R[1]  =     2 * x * y - 2 * z * w;
  R[2]  =     2 * x * z + 2 * y * w;

  R[3]  =     2 * x * y + 2 * z * w;
  R[4]  = 1 - 2 * x * x - 2 * z * z;
  R[5]  =     2 * y * z - 2 * x * w;

  R[6]  =     2 * x * z - 2 * y * w;
  R[7]  =     2 * y * z + 2 * x * w;
  R[8]  = 1 - 2 * x * x - 2 * y * y;

}

//欧拉角 -> 四元数
void euler_to_quaternion(float Yaw, float Pitch, float Roll, float Q[4])
{
  float yaw   = Yaw   * M_PI / 180 ;
  float pitch = Roll  * M_PI / 180 ;
  float roll  = Pitch * M_PI / 180 ;

  float qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
  float qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
  float qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
  float qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
 
  Q[0] = qx;
  Q[1] = qy;
  Q[2] = qz;
  Q[3] = qw;
}


//欧拉角 -> 旋转矩阵
void euler_to_RotationMatrix(float Yaw, float Pitch, float Roll, float R[9])
{
  float yaw   = Yaw   * M_PI / 180 ;
  float pitch = Roll  * M_PI / 180 ;
  float roll  = Pitch * M_PI / 180 ;

  float c1 = cos(yaw);
  float c2 = cos(pitch);
  float c3 = cos(roll);
  float s1 = sin(yaw);
  float s2 = sin(pitch);
  float s3 = sin(roll);

  R[0]  = c1 * c3 + s1 * s2 * s3;
  R[1]  = c3 * s1 * s2 - c1 * s3;
  R[2]  = c2 * s1;

  R[3]  = c2 * s3;
  R[4]  = c2 * c3;
  R[5]  = -s2;

  R[6]  = c1 * s2 * s3 - s1 * c3;
  R[7]  = s1 * s3 + c1 * c3 * s2;
  R[8]  = c1 * c2;
}

