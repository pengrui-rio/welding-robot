


#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <sstream>
#include <fstream>
#include <cstdio>
#include <iostream>
#include <string>
#include <stdio.h>

#include <vector>
#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>

#include <std_msgs/Bool.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

 
 // 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud; 
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloudL;  
typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::Normal> Normal;


extern cv::Mat color_pic, depth_pic;
extern double camera_factor ;
extern double camera_cx ;
extern double camera_cy ;  
extern double camera_fx ;
extern double camera_fy ;

extern tf::TransformListener listener;
extern tf::TransformBroadcaster tf_broadcaster;

using namespace cv;
using namespace std;
using namespace Eigen;
 
void CAD_to_pointcloud(PointCloud::Ptr map_pointcloud);


void CAD_csv_to_pointcloud( string trajectoryInfo_folder_path, PointCloud::Ptr map_pointcloud );


void read_pointcloud_for_icp(string PCD_folder_path, Cloud::Ptr cloud_ptr_in);

void transform_workpiece_pointcloud_model(Cloud::Ptr workpiece_pointcloud_model);

void create_target_pointcloudModel(Cloud::Ptr pointcloud_model_target, Cloud::Ptr workpiece_pointcloud_model);


pcl::IterativeClosestPoint<PointType, PointType> ICP_config(Cloud::Ptr pointcloud_model_target,
                                                            Cloud::Ptr workpiece_pointcloud_model);

void ICP_registration(int iterations, 
                      pcl::IterativeClosestPoint<PointType, PointType> icp,
                      Cloud::Ptr pointcloud_model_target, 
                      Cloud::Ptr workpiece_pointcloud_model);

void ICP_registration(string PCD_folder_path, PointCloud::Ptr map_pointcloud);



void filter(float r, Cloud::Ptr cloud);
