#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>
#include <ctime>

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
#include <pcl/surface/mls.h>
 

using namespace cv;
using namespace std;

void Quaternion_to_RotationMatrix(float x, float y, float z, float w, float R[9]);
void euler_to_quaternion(float Yaw, float Pitch, float Roll, float Q[4]);
void euler_to_RotationMatrix(float Yaw, float Pitch, float Roll, float R[9]);
