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


// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;

//receive robot current pose flag
int receive_pose_flag = 0, process_flag = 0;
float current_x = 0  , current_y = 0    , current_z = 0;
float current_yaw = 0, current_pitch = 0, current_roll = 0;
float current_orientation_x = 0, current_orientation_y = 0, current_orientation_z = 0, current_orientation_w = 0;  

//receive marker current pose
Point3f markerPosition_kinect;
float marker_center_x = 0, marker_center_y = 0, marker_center_z = 0;

void marker_info_Callback(const geometry_msgs::Pose& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{
  markerPosition_kinect.x     = msg.position.x;
  markerPosition_kinect.y     = msg.position.y; 
  markerPosition_kinect.z     = msg.position.z;
}
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
Point3f Base_to_marker_transform(geometry_msgs::Pose Base_kinect, Point3f markerPosition_kinect);
geometry_msgs::Pose compute_base_position(Point3f markerPosition_base);


int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "robot_visuoguiding");
  ros::NodeHandle nh;
  ros::Rate naptime(1000); // use to regulate loop rate 
 
  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber marker_info_sub       = nh.subscribe("marker_info"      , 10, marker_info_Callback);

  ros::Publisher VisuoGuding_publisher = nh.advertise<geometry_msgs::Pose>("VisuoGuding_Pose", 1);


  tf::TransformListener listener;
  tf::TransformBroadcaster tf_broadcaster;

  while (ros::ok()) 
  {
    // listen other coordinates
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/base_link", "/kinect2_rgb_optical_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::Pose Base_kinect;
    Base_kinect.position.x = transform.getOrigin().x();      Base_kinect.position.y = transform.getOrigin().y();      Base_kinect.position.z = transform.getOrigin().z();
    Base_kinect.orientation.x = transform.getRotation().x(); Base_kinect.orientation.y = transform.getRotation().y(); Base_kinect.orientation.z = transform.getRotation().z(); Base_kinect.orientation.w = transform.getRotation().w();
    // cout << "Base_kinect: " << endl << Base_kinect << endl;
    ////////////////////////////////////////////////////////////////////////////////////////////////

 
    geometry_msgs::Pose robot_pose = compute_base_position(  Base_to_marker_transform(Base_kinect, markerPosition_kinect)   );
    VisuoGuding_publisher.publish(robot_pose);
    // cout << "markerPosition_base: " << endl << robot_pose << endl << endl;

    naptime.sleep();
    ros::spinOnce(); //allow data update from callback; 
   }


  ros::spin();
 
}


Point3f Base_to_marker_transform(geometry_msgs::Pose Base_kinect, Point3f markerPosition_kinect)
{
  float R[9] = {};

  // euler_to_RotationMatrix(yaw, roll, pitch, R);
  Eigen::Quaterniond Base_kinect_quaternion(Base_kinect.orientation.w, Base_kinect.orientation.x, Base_kinect.orientation.y, Base_kinect.orientation.z);
  Matrix3d Base_kinect_rotationmatrix = Base_kinect_quaternion.matrix();
  // cout << "Base_kinect_rotationmatrix: " << Base_kinect_rotationmatrix << endl;


  Matrix4d T_B_K;
	T_B_K << Base_kinect_rotationmatrix(0,0), Base_kinect_rotationmatrix(0,1), Base_kinect_rotationmatrix(0,2), Base_kinect.position.x,
           Base_kinect_rotationmatrix(1,0), Base_kinect_rotationmatrix(1,1), Base_kinect_rotationmatrix(1,2), Base_kinect.position.y,
           Base_kinect_rotationmatrix(2,0), Base_kinect_rotationmatrix(2,1), Base_kinect_rotationmatrix(2,2), Base_kinect.position.z,
           0,    0,    0,    1;

  // cout << "T_B_K: "<< endl << T_B_K << endl<< endl;

  Matrix4d T_K_m;
	T_K_m << 1, 0, 0, markerPosition_kinect.x,
           0, 1, 0, markerPosition_kinect.y,
           0, 0, 1, markerPosition_kinect.z,
           0, 0, 0, 1;

  // cout << "T_K_m: "<< endl << T_K_m << endl<< endl;

  Matrix4d T_B_m;
  T_B_m = T_B_K * T_K_m;

  // cout << "T_B_m: "<< endl << T_B_m << endl<< endl;

  Point3f markerPosition_base;
  markerPosition_base.x = T_B_m(0,3);
  markerPosition_base.y = T_B_m(1,3);
  markerPosition_base.z = T_B_m(2,3);

  return markerPosition_base;
}


geometry_msgs::Pose compute_base_position(Point3f markerPosition_base)
{
  // Base -> marker
  Eigen::Matrix3d origin_base_URx;
  origin_base_URx << -1,  0, 0,
                      0, -1, 0, 
                      0,  0, 1;
  
  Eigen::Matrix3d transformed_base_URx;
  transformed_base_URx << 1,  0, 0,
                          0,  0, 1, 
                          0, -1, 0;


  Eigen::Matrix3d rotation_matrix_URx;
  rotation_matrix_URx = origin_base_URx * transformed_base_URx;

  Matrix4d T_Base2Marker;
  T_Base2Marker << rotation_matrix_URx(0,0), rotation_matrix_URx(0,1), rotation_matrix_URx(0,2), -markerPosition_base.x,
                   rotation_matrix_URx(1,0), rotation_matrix_URx(1,1), rotation_matrix_URx(1,2), -markerPosition_base.y,
                   rotation_matrix_URx(2,0), rotation_matrix_URx(2,1), rotation_matrix_URx(2,2),  markerPosition_base.z,
                   0                       , 0                       , 0                       , 1;

  // End -> Camera
  Matrix4d T_E_C;
  T_E_C <<   1,   2.7798e-18,            0,            0,
  -5.57321e-19,            1,  2.71051e-20,    -0.116142,
    1.0842e-19,            0,            1,    0.0708987,
             0,            0,            0,            1;
 
  // camera center -------> marker

  // Base -> End
  Matrix4d T_Base2End;
  T_Base2End = T_Base2Marker * T_E_C.inverse();
  // cout << "T_B_C: " << endl << T_B_C << endl;

  Eigen::Matrix3d End_rotation_matrix_URx;
  End_rotation_matrix_URx << T_Base2End(0,0), T_Base2End(0,1), T_Base2End(0,2),
                             T_Base2End(1,0), T_Base2End(1,1), T_Base2End(1,2),
                             T_Base2End(2,0), T_Base2End(2,1), T_Base2End(2,2);

  Eigen::AngleAxisd End_rotation_vector_URx(End_rotation_matrix_URx);

  ////////////////////////////////////////////////////////////////////////////
  geometry_msgs::Pose pose;

  pose.position.x    = T_Base2End(0,3);
  pose.position.y    = T_Base2End(1,3);
  pose.position.z    = T_Base2End(2,3);
  pose.orientation.x = End_rotation_vector_URx.angle() * End_rotation_vector_URx.axis().x(); 
  pose.orientation.y = End_rotation_vector_URx.angle() * End_rotation_vector_URx.axis().y(); 
  pose.orientation.z = End_rotation_vector_URx.angle() * End_rotation_vector_URx.axis().z(); 
  pose.orientation.w = 0;


  // cout << "welding_trajectory_pose" << pose << endl;

  return pose;
}