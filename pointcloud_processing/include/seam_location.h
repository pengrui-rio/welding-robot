//焊接缝检测：
//1.删除大面积的平面点云
//2.提取边沿，两个面的连接处
//3.找到三条能构成等腰三角形的线，则判断这个区域属于焊接缝的一部分
//4.将所有可能的区域连起来，看能不能构成一条连续的空间曲线
//5.将所有检测出的焊接缝标号



#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>
#include <ctime>
#include <Eigen/Dense>

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

// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud; 
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloudL;  
typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;
typedef pcl::PointXYZ PointType;

typedef pcl::PointCloud<pcl::Normal> Normal;

using namespace cv;
using namespace std;
using namespace Eigen;

void RGBimage_seam_extration(Mat color_pic, Mat depth_pic);


Cloud::Ptr read_pointcloud (PointCloud::Ptr cloud_ptr_show);

vector<Point3f> PointNormal_Computation(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show);

void Delete_SmoothChange_Plane(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector<Point3f> Normal);

vector < vector <float> > Screen_Candidate_Seam(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show);

void Compute_Seam_GeometryCenter(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector < vector <float> > seam_cluster_all, int seam_label);

void Define_StartEnd_Point(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector < vector <float> > seam_cluster_all, int seam_label, char axis, float coordinate);









 