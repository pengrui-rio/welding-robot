#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

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

Cloud::Ptr read_pointcloud (PointCloud::Ptr cloud_ptr_show);

vector<Point3f> allPoint_normal_computation(float sphere_computation, Cloud::Ptr cloud_ptr );

void basic_normal_computation(Cloud::Ptr cloud_ptr, vector<Point3f> cloud_normals, float *basic_normal_x, float *basic_normal_y, float *basic_normal_z);

vector<float> Point_descriptor_computation(PointCloud::Ptr descriptor_cloud, Cloud::Ptr cloud_ptr, vector<Point3f> cloud_normals, float basic_normal_x, float basic_normal_y, float basic_normal_z);

vector<float> Point_variance_computation(Cloud::Ptr cloud_tree_variance, PointCloud::Ptr cloud_tree_variance_show, PointCloud::Ptr descriptor_cloud, vector<float> Dir_descriptor);

void exact_Target_regionPointcloud(PointCloud::Ptr cloud_tree_rm_irrelativePoint, Cloud::Ptr cloud_tree_variance, PointCloud::Ptr cloud_tree_variance_show);

void Exact_seam_region(PointCloud::Ptr cloud_tree_rm_irrelativePoint, PointCloud::Ptr cloud_seamRegion);

vector< vector<int> > Segment_seam_region(PointCloud::Ptr cloud_seamRegion);

void Path_Generation(vector< vector<int> > seg_pointcloud, PointCloud::Ptr cloud_seamRegion, PointCloud::Ptr path_cloud, PointCloud::Ptr path_cloud_showRviz);

