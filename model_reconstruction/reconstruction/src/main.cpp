#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>
#include <reconstruction_transformation.h>
#include "std_msgs/String.h"
#include <ctime>
#include <iostream>
#include <string>
#include <cstdlib>
#include <sstream>
#include <cstring> 

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
// PCL lib
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
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
//namespace enc = sensor_msgs::image_encodings;

// 相机内参
double camera_factor = 1000;
double camera_cx = 311.2325744628906;
double camera_cy = 226.9261474609375;
double camera_fx = 619.9661254882812;
double camera_fy = 619.856201171875;

// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;

//receive robot current pose flag
bool receive_capture_command = false;
int receive_capture_count    = 0;
float current_x = 0  , current_y = 0    , current_z = 0;
float current_yaw = 0, current_pitch = 0, current_roll = 0;
 

//callback function:
void color_Callback(const sensor_msgs::ImageConstPtr& color_msg)
{
  try
  {
    // cv::imshow("color_view", cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8)->image);
    color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);    
  }
  catch (cv_bridge::Exception& e )
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
  }
  color_pic = color_ptr->image;
  // cv::imshow("color_view", color_pic);
 
  waitKey(1); 
}
void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
  try
  {
    // cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image);
    depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1); 
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
  }
  depth_pic = depth_ptr->image;
  // cv::imshow("depth_pic", depth_pic);


  waitKey(1);
}
void capture_command_Callback(const std_msgs::String::ConstPtr& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{
  ROS_INFO("I heard the pose from the capture pointcloud command"); 
  receive_capture_command = true;
  receive_capture_count++;
}
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void show_pointcloud_Rviz(int show_Pointcloud_timeMax, 
                          PointCloud::Ptr show_Rviz_cloud, 
                          sensor_msgs::PointCloud2 pub_pointcloud, 
                          ros::Publisher pointcloud_publisher);

void publish_pointcloud_Rviz(string coordinate, 
                             PointCloud::Ptr pointloud, 
                             sensor_msgs::PointCloud2 pub_pointcloud, 
                             ros::Publisher pointcloud_publisher);

string create_pointcloud_storageFolder_sendPath(ros::Publisher pointcloud_storageFolder_pub, 
                                                ros::Rate naptime);

 
int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "model_reconstruction");
  ros::NodeHandle nh;
  ros::Rate naptime(1000); // use to regulate loop rate 

  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("send_capture_command", 1, capture_command_Callback);
  image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
  image_transport::Subscriber depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_Callback);///camera/depth/image_rect_raw
 
  //publisher:
  ros::Publisher pointcloud_storageFolder_pub          = nh.advertise<std_msgs::String>("pointcloud_storageFolder", 1);
  ros::Publisher Moveit_path_publisher                 = nh.advertise<geometry_msgs::Pose>("Moveit_motion_Path", 1);
  ros::Publisher pointcloud_publisher                  = nh.advertise<sensor_msgs::PointCloud2>("processing_pointcloud", 1);
  ros::Publisher camera_pointcloud_publisher           = nh.advertise<sensor_msgs::PointCloud2>("camera_pointcloud", 1);
  ros::Publisher cam_pc_transform_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("cam_pc_transform_pointcloud", 1);
  ros::Publisher map_pointcloud_publisher              = nh.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 1);
  ros::Publisher vis_pub                               = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  //pointcloud msgs: 
  sensor_msgs::PointCloud2 pub_camera_pointcloud;
  sensor_msgs::PointCloud2 pub_cam_pc_transform_pointcloud;
  sensor_msgs::PointCloud2 pub_map_pointcloud;
  sensor_msgs::PointCloud2 pub_path; 
  sensor_msgs::PointCloud2 pub_pointcloud;
 
  //pointcloud:
  PointCloud::Ptr camera_pointcloud            (new PointCloud);
  PointCloud::Ptr cam_pc_transform_pointcloud  (new PointCloud);
  PointCloud::Ptr map_pointcloud               (new PointCloud);
  Cloud::Ptr      cloud_ptr                    (new Cloud);   
  Cloud::Ptr      cloud_ptr_filter             (new Cloud); 
  Cloud::Ptr      cloud_ptr_input              (new Cloud);

  //tf:
  tf::TransformListener listener;
  tf::TransformBroadcaster tf_broadcaster;

  //create pointcloud dataset and send the folder path:
  // string dataset_folder_path = create_pointcloud_storageFolder_sendPath(pointcloud_storageFolder_pub, naptime);

  ///////////////////////////////////////////////////////////////////////////////

  while (ros::ok()) 
  {
    // break;
    tf::StampedTransform transform_baselink2tool0;
    tf::StampedTransform transform_tool02torch;
    try
    {
      listener.lookupTransform("/base_link", "/tool0", ros::Time(0)        , transform_baselink2tool0);
      listener.lookupTransform("/tool0"    , "/welding_torch", ros::Time(0), transform_tool02torch);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    //////////////////////////////////////////////////////////////////////////////////////
    //1.解析raw data
    analyze_realsense_data(camera_pointcloud);
    publish_pointcloud_Rviz("camera_color_optical_frame",
                             camera_pointcloud, 
                             pub_camera_pointcloud, 
                             camera_pointcloud_publisher);                          
    //////////////////////////////////////////////////////////////////////////////////////
    //2.变换点云坐标系
    coordinate_transformation(transform_baselink2tool0, 
                              camera_pointcloud, 
                              cam_pc_transform_pointcloud, 
                              cloud_ptr);
    publish_pointcloud_Rviz("base_link", 
                             cam_pc_transform_pointcloud,  
                             pub_cam_pc_transform_pointcloud, 
                             cam_pc_transform_pointcloud_publisher);
    //////////////////////////////////////////////////////////////////////////////////////
    // //3.拼接基座坐标系下的点云，重建model
    // model_3D_reconstruction(receive_capture_command,
    //                         receive_capture_count,
    //                         dataset_folder_path,
    //                         realsense_position_acquisition(transform_baselink2tool0), 
    //                         cam_pc_transform_pointcloud,
    //                         map_pointcloud);
    // publish_pointcloud_Rviz("base_link", 
    //                          map_pointcloud, 
    //                          pub_map_pointcloud, 
    //                          map_pointcloud_publisher);
    //////////////////////////////////////////////////////////////////////////////////////
     
    camera_pointcloud->points.clear();
    cam_pc_transform_pointcloud->points.clear();
    cloud_ptr->points.clear();

    ros::spinOnce(); //allow data update from callback; 
    // naptime.sleep(); // wait for remainder of specified period; 
  }

  // ros::spin();

  return 0;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void publish_pointcloud_Rviz(string coordinate, 
                             PointCloud::Ptr pointloud, 
                             sensor_msgs::PointCloud2 pub_pointcloud, 
                             ros::Publisher pointcloud_publisher)
{
    pcl::toROSMsg(*pointloud, pub_pointcloud);
    pub_pointcloud.header.frame_id = coordinate;// 
    pub_pointcloud.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(pub_pointcloud);
}
 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void show_pointcloud_Rviz(int show_Pointcloud_timeMax, 
                          PointCloud::Ptr show_Rviz_cloud, 
                          sensor_msgs::PointCloud2 pub_pointcloud, 
                          ros::Publisher pointcloud_publisher)
{
  for(float i = 0; i < show_Pointcloud_timeMax; i++)
  {
    pcl::toROSMsg(*show_Rviz_cloud, pub_pointcloud);
    pub_pointcloud.header.frame_id = "base_link";
    pub_pointcloud.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(pub_pointcloud);
  }
  show_Rviz_cloud->points.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//一堆字符转string 字符串：
//char aaa[] = {a,b,c,d...}
//string str = aaa;
string create_pointcloud_storageFolder_sendPath(ros::Publisher pointcloud_storageFolder_pub, ros::Rate naptime)
{
  // 基于当前系统的当前日期/时间
  time_t now = time(0);
  char* dt = ctime(&now);
  cout << "本地日期和时间：" << dt << endl;
  char date_time[] = {dt[0], dt[1], dt[2], '_', dt[4], dt[5], dt[6], '_', dt[9], '_', dt[11], dt[12], '_', dt[14], dt[15], '_', dt[17], dt[18], '\0'};
  string folder_name = date_time;
 
  string folderPath = "/home/rick/Documents/a_system/src/pointcloud_dataset/" ; 
  string dataset_folder_path = folderPath + folder_name;
  cout << "nPath:" << dataset_folder_path << endl;

  string command;
  command = "mkdir -p " + dataset_folder_path;  
  system(command.c_str());

  std_msgs::String msg;
  std::stringstream ss;
  ss << dataset_folder_path;
  msg.data = ss.str();
  ROS_INFO("%s", msg.data.c_str());

  for(int i = 0; i < 5000; i ++)
  {
    ROS_INFO("%s", msg.data.c_str());
    pointcloud_storageFolder_pub.publish(msg);
    naptime.sleep();
  }

  return dataset_folder_path;
}
 