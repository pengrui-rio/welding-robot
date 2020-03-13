//运动规划：
//1.确定焊接方向
//2.分割焊接缝
//3.计算焊接点
//4.为每个焊接点计算旋转方向





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
#include <pcl/kdtree/kdtree_flann.h>//搜索方法
#include <boost/thread/thread.hpp>

#include <pcl/features/boundary.h>

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


// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud; 
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloudL;  
typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;
typedef pcl::PointXYZ PointType;

typedef pcl::PointCloud<pcl::Normal> Normal;

using namespace cv;
using namespace std;
using namespace Eigen;


vector <float> compute_Points_disSum(Cloud::Ptr cloud_ptr);

float Point_DisSum_min_Compute(vector <float> Point_DisSum);

vector <float> cloud_GeometryCenter_pack(float Point_DisSum_min_index, Cloud::Ptr cloud_ptr);

vector <float> Compute_Segment_GeometryCenter(Cloud::Ptr cloud_ptr);

float Distance_two_Points(pcl::PointXYZ p1, pcl::PointXYZ p2);

vector<pcl::PointXYZ> Points_Exchange(pcl::PointXYZ p1, pcl::PointXYZ p2);

Cloud::Ptr Create_SeamCloud( Cloud::Ptr cloud_ptr );

Cloud::Ptr Output_Boundary_SeamCloud(Cloud::Ptr seam_cloud);

Cloud::Ptr Delete_noiseBoundary(Cloud::Ptr boundPoints);

void cloud_ptr_show_creation(Cloud::Ptr seam_edge, PointCloud::Ptr cloud_ptr_show);

vector<int> FindAllIndex_Around_OnePoint(Cloud::Ptr seam_edge, float i, float radius);

vector<Point3f> Distance_and_Index(Cloud::Ptr seam_edge, vector<int> pointIdxRadiusSearch);

float Distance_Points_max_index_Compute(vector<Point3f> Distance_Points);

Point3f Compute_Vector_TwoPoints(pcl::PointXYZ p1, pcl::PointXYZ p2);

float Compute_Included_Angle(Point3f vector1, Point3f vector2);

vector<float> Find_relevantPoint_onTheOherCurve(vector<Point3f> Distance_Points, float Distance_Points_max_index, Cloud::Ptr seam_edge, vector<int> pointIdxRadiusSearch);

pcl::PointXYZ Compute_Single_PathPoint(Cloud::Ptr seam_edge, vector<int> pointIdxRadiusSearch, float right_point_index);

void DownSample_DeleteNoisePoint(Cloud::Ptr Path_Cloud, float radius);

Cloud::Ptr Merge_NearPoints(Cloud::Ptr Path_Cloud, float radius);

float Included_Value_TwoPoints(Point3f vector1, Point3f vector2);

Cloud::Ptr Order_PathPoints_Cloud(Cloud::Ptr Path_Cloud_filtered, float radius);

void Show_Ordered_PathPoints(Cloud::Ptr Path_Cloud_final, Cloud::Ptr cloud_ptr_origin, PointCloud::Ptr cloud_ptr_show);

Cloud::Ptr Compute_All_PathPoints(Cloud::Ptr seam_edge);

void push_point_showCloud(Cloud::Ptr seam_edge, PointCloud::Ptr cloud_ptr_show);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Cloud::Ptr Extract_Seam_edge(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show);

Cloud::Ptr PathPoint_Position_Generation(Cloud::Ptr seam_edge, Cloud::Ptr cloud_ptr_origin, PointCloud::Ptr cloud_ptr_show);

vector<Point3f> PathPoint_Orientation_Generation(Cloud::Ptr PathPoint_Position, Cloud::Ptr cloud_ptr_origin, Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show);



