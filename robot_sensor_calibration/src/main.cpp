#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>
#include <algorithm.h>
#include <transformation.h>

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
//namespace enc = sensor_msgs::image_encodings;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 320;
const double camera_cy = 240;
const double camera_fx = 670.588;
const double camera_fy = 670.588;

// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;

//receive robot current pose flag
int receive_pose_flag = 0, process_flag = 0;
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
  waitKey(1);
}
void robot_currentpose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{

  ROS_INFO("I heard the pose from the robot"); 
  ROS_INFO("the position(x,y,z) is %f , %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  ROS_INFO("the orientation(yaw, pitch, roll) is %f , %f, %f ", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  ROS_INFO("the time we get the pose is %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec);

  current_x     = msg->pose.position.x;
  current_y     = msg->pose.position.y; 
  current_z     = msg->pose.position.z;

  current_yaw   = msg->pose.orientation.x;
  current_pitch = msg->pose.orientation.y;
  current_roll  = msg->pose.orientation.z;

  process_flag  = msg->pose.orientation.w;
  receive_pose_flag = 1;
  
  cout << " \n" << endl; //add two more blank row so that we can see the message more clearly
}
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void seam_detection(ros::Rate naptime, Cloud::Ptr cloud_ptr, ros::Publisher path_publisher, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher);
void analyze_realsense_data(PointCloud::Ptr cloud);
void coordinate_transformation(PointCloud::Ptr camera_pointcloud, PointCloud::Ptr map_pointcloud, Cloud::Ptr cloud_ptr);


int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::Rate naptime(10); // use to regulate loop rate 

  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("robot_currentpose", 10, robot_currentpose_Callback);
  image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
  image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image_rect_raw", 1, depth_Callback);
 
  //publisher:
  ros::Publisher path_publisher              = nh.advertise<geometry_msgs::Pose>("motion_Path", 1);
  ros::Publisher pointcloud_publisher        = nh.advertise<sensor_msgs::PointCloud2>("generated_pc", 1);
  ros::Publisher camera_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("camera_pointcloud", 1);
  ros::Publisher map_pointcloud_publisher    = nh.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 1);
 
  //pointcloud msgs: 
  sensor_msgs::PointCloud2 pub_camera_pointcloud;
  sensor_msgs::PointCloud2 pub_map_pointcloud;
  sensor_msgs::PointCloud2 pub_path;
  sensor_msgs::PointCloud2 pub_pointcloud;
 
  //pointcloud:
  PointCloud::Ptr camera_pointcloud (new PointCloud);
  PointCloud::Ptr map_pointcloud    (new PointCloud);
  Cloud::Ptr      cloud_ptr (new Cloud);


  while (ros::ok()) 
  {
    analyze_realsense_data(camera_pointcloud);

    pcl::toROSMsg(*camera_pointcloud, pub_camera_pointcloud);
    pub_camera_pointcloud.header.frame_id = "camera_depth_optical_frame"; // camera_depth_optical_frame
    pub_camera_pointcloud.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(pub_camera_pointcloud);


    camera_pointcloud->points.clear();

    ros::spinOnce(); //allow data update from callback; 
    // naptime.sleep(); // wait for remainder of specified period; 
  }


  ros::spin();
 
}



void analyze_realsense_data(PointCloud::Ptr cloud)
{
  // 遍历深度图
  for (int m = 0; m < depth_pic.rows; m++)  //480
  {
    for (int n = 0; n < depth_pic.cols; n++) //640
    {
      // 获取深度图中(m,n)处的值
      float d = depth_pic.ptr<float>(m)[n]; 
      // d 可能没有值，若如此，跳过此点
      if (d == 0)
          continue;

      // d 存在值，则向点云增加一个点 
      pcl::PointXYZRGB p, p_z, p_x, p_y;
      pcl::PointXYZRGB p_transform;

      // 计算这个点的空间坐标
      p.z = double(d) / camera_factor ;
      p.x = (n - camera_cx) * p.z / camera_fx; 
      p.y = (m - camera_cy) * p.z / camera_fy;

      // 从rgb图像中获取它的颜色
      // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
      p.b = color_pic.ptr<uchar>(m)[n*3];
      p.g = color_pic.ptr<uchar>(m)[n*3+1];
      p.r = color_pic.ptr<uchar>(m)[n*3+2];

      cloud->points.push_back( p );        
    }
  }

  // cout << 480 * 0.475 / 0.34 << endl;

}
