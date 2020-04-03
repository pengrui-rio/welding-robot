


#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>
#include <Eigen/Dense>
#include <string>
#include <dirent.h>
#include <seam_location.h>
#include <motion_planning.h>

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

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>//点云文件pcd 读写
#include <pcl/visualization/cloud_viewer.h>//点云可视化
#include <pcl/visualization/pcl_visualizer.h>// 高级可视化点云类
#include <pcl/features/normal_3d.h>//法线特征
#include <pcl/kdtree/kdtree_flann.h>//搜索方法
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
#include <pcl/surface/mls.h>


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
  

vector< geometry_msgs::Pose > trajectory_6DOF_generation(pcl::PointXYZ realsense_position, 
                                                         Cloud::Ptr cloud_ptr, 
                                                         Cloud::Ptr cloud_ptr_modelSeam, 
  
                                                         bool &trajectoryPlanning_flag, 
                                                         int  &receive_capture_count,

                                                         sensor_msgs::PointCloud2 pub_pointcloud, 
                                                         ros::Publisher pointcloud_publisher, 
                                                         ros::Publisher Welding_Trajectory_publisher,
                                                         
                                                         ros::Rate naptime);



bool welding_seam_location( Cloud::Ptr cloud_ptr, 
                            pcl::PointXYZ realsense_position, 
                            sensor_msgs::PointCloud2 pub_pointcloud, 
                            ros::Publisher pointcloud_publisher);


vector< geometry_msgs::Pose > trajectory_planning(Cloud::Ptr cloud_ptr_modelSeam,
                                                  vector< pcl::PointXYZ > all_realsense_position,
                                                  sensor_msgs::PointCloud2 pub_pointcloud, 
                                                  ros::Publisher pointcloud_publisher,
                                                  ros::Publisher Welding_Trajectory_publisher,
                                                  ros::Rate naptime);


void input_pointcloud_filter(Cloud::Ptr cloud_ptr);


void integrate_allsingle_pointcloudFrame(Cloud::Ptr cloud_ptr, 
                                         Cloud::Ptr cloud_ptr_modelSeam, 
                                         sensor_msgs::PointCloud2 pub_pointcloud, 
                                         ros::Publisher pointcloud_publisher);



int count_pointcloud_frameNum(string dataset_folder_path);


pcl::PointXYZ read_realtime_pointcloud_frame( string dataset_folder_path,
                                              int pointcloud_frameNum,
                                              int receive_capture_count,
                                              int &process_frame_count,
                                              bool &trajectoryPlanning_flag,
                                              Cloud::Ptr cloud_ptr);

void build_model_pointcloud(string dataset_folder_path, 
                            int pointcloud_frameNum,
                            PointCloud::Ptr model_pointcloud);




void show_pointcloud_Rviz(int show_Pointcloud_timeMax, 
                          PointCloud::Ptr show_Rviz_cloud, 
                          sensor_msgs::PointCloud2 pub_pointcloud, 
                          ros::Publisher pointcloud_publisher);

void publish_pointcloud_Rviz( string coordinate, 
                              PointCloud::Ptr pointloud, 
                              sensor_msgs::PointCloud2 pub_pointcloud, 
                              ros::Publisher pointcloud_publisher);

bool processing_frame_ornot(Cloud::Ptr cloud_ptr, 
                            int show_Pointcloud_timeMax, 
                            PointCloud::Ptr cloud_ptr_show, 
                            sensor_msgs::PointCloud2 pub_pointcloud, 
                            ros::Publisher pointcloud_publisher);

