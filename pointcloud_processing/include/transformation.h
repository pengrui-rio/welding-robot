#include <math.h>
#include <vector>
#include <Eigen/Dense>


#include <std_msgs/Bool.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

 
 // 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud; 
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloudL;  
typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::Normal> Normal;



using namespace cv;
using namespace std;
using namespace Eigen;
 
pcl::PointXYZ camera_to_base_transform(geometry_msgs::Pose Base_End, pcl::PointXYZ Cam_Object);

pcl::PointXYZ realsense_position_acquisition(geometry_msgs::Pose Base_End);




void rotate_z(float x, float y, float z, float angle, float* x_output, float* y_output, float* z_output) ;
void rotate_x(float x, float y, float z, float angle, float* x_output, float* y_output, float* z_output)   ;
void rotate_y(float x, float y, float z, float angle, float* x_output, float* y_output, float* z_output)   ;