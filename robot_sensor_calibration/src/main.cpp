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
 
void EyeinHand_compute_TEC(void);
void EyeinHand_Result_validation(void);
void EyetoHand_compute_TBC(void);


int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "robot_sensor_calibration");
  ros::NodeHandle nh;
  ros::Rate naptime(10); // use to regulate loop rate 
 
  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("robot_currentpose", 10, robot_currentpose_Callback);

  // EyetoHand_compute_TBC();
  EyeinHand_compute_TEC();
  // EyeinHand_Result_validation();
  cout << " \n" << endl; 


  tf::TransformListener listener;
  tf::TransformBroadcaster tf_broadcaster;

  while (ros::ok()) 
  {
    // // listen other coordinates
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/tool0", "/camera_color_optical_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    cout << "CamToOutput.getOrigin: " << transform.getOrigin().x() << " "
                                      << transform.getOrigin().y() << " " 
                                      << transform.getOrigin().z()<< endl;
    cout << "CamToOutput.getrotation: " << transform.getRotation().x() << " " 
                                        << transform.getRotation().y() << " " 
                                        << transform.getRotation().z() << " " 
                                        << transform.getRotation().w() << endl<< endl;


    //broadcast ground truth of AR marker:
    double px = 0.056;//double px = -0.04;
    double py = 0.328;
    double pz = 0.001;
    float T_o_B_q[4];
    euler_to_quaternion(90, 0, 0, T_o_B_q);
    double qx = T_o_B_q[0];
    double qy = T_o_B_q[1];
    double qz = T_o_B_q[2];
    double qw = T_o_B_q[3];

    tf::Quaternion rotation (qx,qy,qz,qw);
    tf::Vector3 origin (px,py,pz);
    tf::Transform t (rotation, origin);

    std::string markerFrame = "GT_ar_marker_";
    std::stringstream out;
    out << 1;
    std::string id_string = out.str();
    markerFrame += id_string;
    tf::StampedTransform GT_Marker (t, ros::Time::now(), "/base_link", markerFrame.c_str());
    tf_broadcaster. sendTransform(GT_Marker);









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


void EyeinHand_compute_TEC(void)
{
  //Base -> End:
  float T_B_E_Q[4];
  euler_to_quaternion(20, -170, 0, T_B_E_Q);
  T_B_E_Q[0] = -0.999961879971;
  T_B_E_Q[1] = -0.00503657323601;
  T_B_E_Q[2] = -0.00243954619579;
  T_B_E_Q[3] =  0.00670224956124;

  float T_B_E_r[9];
  Quaternion_to_RotationMatrix(T_B_E_Q[0], T_B_E_Q[1], T_B_E_Q[2], T_B_E_Q[3], T_B_E_r);
  // euler_to_RotationMatrix(0, -180, 0, T_B_E_r);
  Matrix4d T_B_E; // Base -> End
	T_B_E << T_B_E_r[0], T_B_E_r[1], T_B_E_r[2], 0.000,
           T_B_E_r[3], T_B_E_r[4], T_B_E_r[5], 0.2506,
           T_B_E_r[6], T_B_E_r[7], T_B_E_r[8], 0.4517,
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
	T_C_o << T_C_o_r[0], T_C_o_r[1], T_C_o_r[2], -0.01479443,
           T_C_o_r[3], T_C_o_r[4], T_C_o_r[5], -0.0157338,
           T_C_o_r[6], T_C_o_r[7], T_C_o_r[8],  0.423117,
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

  // T_E_C: 
  //   0.999937  -0.0100401 -0.00494651  -0.0247213
  //   0.0101056     0.99986   0.0133794  -0.0615432
  // 0.00481148  -0.0134286    0.999898    0.028287
  //         -0           0           0           1


}

void EyeinHand_Result_validation(void)
{
  //Base -> End:
  float T_B_E_Q[4];
  T_B_E_Q[0] = -0.999951264231;
  T_B_E_Q[1] = -0.00534872895094;
  T_B_E_Q[2] = -0.00359072703145;
  T_B_E_Q[3] = 0.0074811055935;
  float T_B_E_r[9];
  Quaternion_to_RotationMatrix(T_B_E_Q[0], T_B_E_Q[1], T_B_E_Q[2], T_B_E_Q[3], T_B_E_r);
  Matrix4d T_B_E; // Base -> End
	T_B_E << T_B_E_r[0], T_B_E_r[1], T_B_E_r[2], 0.0187,
           T_B_E_r[3], T_B_E_r[4], T_B_E_r[5], 0.2508,
           T_B_E_r[6], T_B_E_r[7], T_B_E_r[8], 0.4520,
           0,          0,          0,          1;
  cout << "T_B_E: " << endl << T_B_E << endl;


  // End -> Camera
  Matrix4d T_E_C;
	T_E_C << 0.999917,   -0.0106432, -0.00726122, -0.0256469,
           0.0107508,   0.999831,   0.0149231,  -0.0600286,
           0.00710116, -0.0149999,  0.999862,    0.032314,
           0,           0,          0,           1;
  cout << "T_E_C: " << endl << T_E_C << endl;


  // Base -> Camera
  Matrix4d T_B_C;
  T_B_C = T_B_E * T_E_C;
  cout << "T_B_C: " << endl << T_B_C << endl;


  // Camera -> object  0.0691944 -0.0164191
	Matrix4d T_C_o; 
	T_C_o << 1, 0, 0, 0.0691944,
           0, 1, 0, -0.0164191,
           0, 0, 1, 0.4194,
           0, 0, 0, 1;
  cout << "T_C_o: " << endl << T_C_o << endl;
  // cout << endl;


  // result by calibration:
  Matrix4d T_R;
  T_R = T_B_C * T_C_o;
  cout << "T_R: " << endl << T_R << endl;

}

