#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>
#include <CAD2pointcloud_transformation.h>
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
 
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void workpiece_3D_model_2_workpiece_3D_model_pointcloud(Cloud::Ptr cloud_point, 
                                                        PointCloud::Ptr pointloud_point);

void model_target_2_model_target_pointcloud(Cloud::Ptr cloud_point, 
                                            PointCloud::Ptr pointloud_point);

void show_pointcloud_Rviz(int show_Pointcloud_timeMax, 
                          PointCloud::Ptr show_Rviz_cloud, 
                          sensor_msgs::PointCloud2 pub_pointcloud, 
                          ros::Publisher pointcloud_publisher);

void publish_pointcloud_Rviz(string coordinate, 
                             PointCloud::Ptr pointloud, 
                             sensor_msgs::PointCloud2 pub_pointcloud, 
                             ros::Publisher pointcloud_publisher);

  
int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "model_reconstruction");
  ros::NodeHandle nh;
  ros::Rate naptime(1000); // use to regulate loop rate 
  
  //publisher:
  ros::Publisher model_target_pointcloud_publisher       = nh.advertise<sensor_msgs::PointCloud2>("model_target_pointcloud", 1);
  ros::Publisher workpiece_3D_model_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("workpiece_3D_model_pointcloud", 1);
  ros::Publisher vis_pub                                 = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  //pointcloud msgs: 
 
  sensor_msgs::PointCloud2 pub_model_target_pointcloud;
  sensor_msgs::PointCloud2 pub_workpiece_3D_model_pointcloud;
  
  // CAD_csv_to_pointcloud("/home/rick/Documents/a_system/src/CAD2pointcloud/stl_file/cylinder.csv", map_pointcloud);
  ///////////////////////////////////////////////////////////////////////////////
  Cloud::Ptr   workpiece_3D_model  (new Cloud);  
  PointCloud::Ptr workpiece_3D_model_pointcloud (new PointCloud); 

  Cloud::Ptr   model_target        (new Cloud);    Cloud::Ptr   model_target_temp       (new Cloud);  
  PointCloud::Ptr model_target_pointcloud       (new PointCloud);

  read_pointcloud_for_icp("/home/rick/Documents/a_system/src/CAD2pointcloud/stl_file/ICP_straight.pcd", workpiece_3D_model);
  // create_target_pointcloudModel(model_target, workpiece_3D_model);
  transform_workpiece_pointcloud_model(workpiece_3D_model);

  read_pointcloud_for_icp("/home/rick/Documents/a_system/src/CAD2pointcloud/stl_file/1_frame.pcd", model_target);
  *model_target_temp += *model_target;  model_target->points.clear();
  read_pointcloud_for_icp("/home/rick/Documents/a_system/src/CAD2pointcloud/stl_file/2_frame.pcd", model_target);
  *model_target_temp += *model_target;

  workpiece_3D_model_2_workpiece_3D_model_pointcloud(workpiece_3D_model, workpiece_3D_model_pointcloud);
  publish_pointcloud_Rviz("base_link", 
                            workpiece_3D_model_pointcloud, 
                            pub_workpiece_3D_model_pointcloud, 
                            workpiece_3D_model_pointcloud_publisher);

  model_target_2_model_target_pointcloud(model_target_temp, model_target_pointcloud);
  publish_pointcloud_Rviz("base_link", 
                            model_target_pointcloud, 
                            pub_model_target_pointcloud, 
                            model_target_pointcloud_publisher);

  pcl::IterativeClosestPoint<PointType, PointType> icp = ICP_config(model_target_temp, workpiece_3D_model);
  
  int iterations = 1;  // Default number of ICP iterations

  while (ros::ok()) 
  {
    ICP_registration(iterations, icp, model_target_temp, workpiece_3D_model);
    iterations++;

    workpiece_3D_model_2_workpiece_3D_model_pointcloud(workpiece_3D_model, workpiece_3D_model_pointcloud);
    publish_pointcloud_Rviz("base_link", 
                             workpiece_3D_model_pointcloud, 
                             pub_workpiece_3D_model_pointcloud, 
                             workpiece_3D_model_pointcloud_publisher);

    model_target_2_model_target_pointcloud(model_target_temp, model_target_pointcloud);
    publish_pointcloud_Rviz("base_link", 
                             model_target_pointcloud, 
                             pub_model_target_pointcloud, 
                             model_target_pointcloud_publisher);
 
 
    ros::spinOnce(); //allow data update from callback; 
    // naptime.sleep(); // wait for remainder of specified period; 
  }

  // ros::spin();

  return 0;
}


 


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void workpiece_3D_model_2_workpiece_3D_model_pointcloud(Cloud::Ptr cloud_point, PointCloud::Ptr pointloud_point)
{ 
  pointloud_point->points.clear();
  for(float j = 0; j < cloud_point->points.size(); j++)
  {
    pcl::PointXYZRGB p; 
    p.x = cloud_point->points[j].x;
    p.y = cloud_point->points[j].y;
    p.z = cloud_point->points[j].z;
    p.b = 0;
    p.g = 0;
    p.r = 200;

    pointloud_point->points.push_back( p );    
  }
}

void model_target_2_model_target_pointcloud(Cloud::Ptr cloud_point, PointCloud::Ptr pointloud_point)
{ 
  pointloud_point->points.clear();
  for(float j = 0; j < cloud_point->points.size(); j++)
  {
    pcl::PointXYZRGB p; 
    p.x = cloud_point->points[j].x;
    p.y = cloud_point->points[j].y;
    p.z = cloud_point->points[j].z;
    p.b = 200;
    p.g = 200;
    p.r = 200;

    pointloud_point->points.push_back( p );    
  }
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

