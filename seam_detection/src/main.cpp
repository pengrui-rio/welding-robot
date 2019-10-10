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
const double camera_factor = 985;
const double camera_cx = 320;
const double camera_cy = 240;
const double camera_fx = 615.899;
const double camera_fy = 616.468;

//receive robot current pose flag
int receive_pose_flag = 0;
float current_x = 0  , current_y = 0    , current_z = 0;
float current_yaw = 0, current_pitch = 0, current_roll = 0;
 
 
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

  receive_pose_flag = 1;

  cout << " \n" << endl; //add two more blank row so that we can see the message more clearly
}
 

void show_pointcloud_Rviz(int show_Pointcloud_timeMax, PointCloud::Ptr cloud, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher)
{
  PointCloud::Ptr show_Rviz_cloud (new PointCloud);

  for(float i = 0; i < cloud->points.size(); i++)
  {
    pcl::PointXYZRGB p;
    p.x = cloud->points[i].x; 
    p.y = cloud->points[i].y;
    p.z = cloud->points[i].z;
    p.b = cloud->points[i].b; 
    p.g = cloud->points[i].g;
    p.r = cloud->points[i].r;

    show_Rviz_cloud->points.push_back( p );    
  }

  for(float i = 0; i < show_Pointcloud_timeMax; i++)
  {
    pcl::toROSMsg(*show_Rviz_cloud, pub_pointcloud);
    pub_pointcloud.header.frame_id = "base_link";
    pub_pointcloud.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(pub_pointcloud);
  }
  show_Rviz_cloud->points.clear();
}


void seam_detection(ros::Rate naptime, ros::Publisher path_publisher, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher)
{
  int show_Pointcloud_timeMax = 2;
  float sphere_computation = 0.005;

  //1.读入原始pointcloud
  PointCloud::Ptr cloud_ptr_show (new PointCloud);
  Cloud::Ptr cloud_ptr = read_pointcloud(cloud_ptr_show);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  ////////////////////////////////////////////////////////////

  //2.算出所有点的法向量
  vector<Point3f> cloud_normals = allPoint_normal_computation(sphere_computation, cloud_ptr);
  //////////////////////////////////////////////////////////

  //3.计算基准法向量，并返回提出nan点后的点云
  float basic_normal_x = 0, basic_normal_y = 0, basic_normal_z = 0;
  basic_normal_computation(cloud_ptr, cloud_normals, &basic_normal_x, &basic_normal_y, &basic_normal_z );
  //////////////////////////////////////////////////////////

  //4.计算每个点的方向描述子
  PointCloud::Ptr descriptor_cloud (new PointCloud);
  vector<float> Dir_descriptor = Point_VarianceDescriptor_computation(sphere_computation, descriptor_cloud, cloud_ptr, cloud_normals, basic_normal_x, basic_normal_y, basic_normal_z);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, descriptor_cloud, pub_pointcloud, pointcloud_publisher);
  ////////////////////////////////////////////////////////////

  // //5.计算每个点在邻域内的方差
  // Cloud::Ptr cloud_tree_variance (new Cloud);
  // PointCloud::Ptr cloud_tree_variance_show (new PointCloud);
  // vector<float> variance_descriptor = Point_variance_computation(cloud_tree_variance, cloud_tree_variance_show, descriptor_cloud, Dir_descriptor);
  // show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_tree_variance_show, pub_pointcloud, pointcloud_publisher);
  // ////////////////////////////////////////////////////////////

  // //6.提取目标区域点云，可能有干扰
  // PointCloud::Ptr cloud_tree_rm_irrelativePoint (new PointCloud);
  // exact_Target_regionPointcloud(cloud_tree_rm_irrelativePoint, cloud_tree_variance, cloud_tree_variance_show);
  // show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_tree_rm_irrelativePoint, pub_pointcloud, pointcloud_publisher);
  // ////////////////////////////////////////////////////////////

  // //7.利用欧式聚类剔除干扰区域，这里选取聚类数目最多的区域作为最终目标区域
  // PointCloud::Ptr cloud_seamRegion (new PointCloud);
  // Exact_seam_region(cloud_tree_rm_irrelativePoint, cloud_seamRegion);
  // show_pointcloud_Rviz(50*show_Pointcloud_timeMax, cloud_seamRegion, pub_pointcloud, pointcloud_publisher);
  // ////////////////////////////////////////////////////////////

  // //8.分割seam区域并确定path主方向
  // vector< vector<int> > seg_pointcloud = Segment_seam_region(cloud_seamRegion);
  // show_pointcloud_Rviz(50*show_Pointcloud_timeMax, cloud_seamRegion, pub_pointcloud, pointcloud_publisher);
  // ////////////////////////////////////////////////////////////

  // //9.通过梯度下降优化出最佳运动path
  // PointCloud::Ptr path_cloud (new PointCloud);
  // PointCloud::Ptr path_cloud_showRviz (new PointCloud);
  // Path_Generation(seg_pointcloud, cloud_seamRegion, path_cloud, path_cloud_showRviz);
  // show_pointcloud_Rviz(100*show_Pointcloud_timeMax, cloud_seamRegion, pub_pointcloud, pointcloud_publisher);
  // show_pointcloud_Rviz(100*show_Pointcloud_timeMax, path_cloud_showRviz, pub_pointcloud, pointcloud_publisher);
  // ////////////////////////////////////////////////////////////
  // cout << "3D path is generated !!!!!!!!!" << endl;

  // geometry_msgs::Pose path_point;
  // for(int i = 0; i < path_cloud->points.size(); i++)
  // {
  //   pcl::PointXYZRGB p, p_z, p_x, p_y;

  //   p.x    = path_cloud->points[i].x; 
  //   p.y    = path_cloud->points[i].y; 
  //   p.z    = path_cloud->points[i].z; 

  //   // rotate_y(p.x,   p.y,   p.z,   current_roll  , &p_y.x, &p_y.y, &p_y.z);
  //   // rotate_x(p_y.x, p_y.y, p_y.z, current_pitch , &p_x.x, &p_x.y, &p_x.z);
  //   // rotate_z(p_x.x, p_x.y, p_x.z, current_yaw   , &p_z.x, &p_z.y, &p_z.z); 

  //   // path_point.position.x = current_x + p_z.x ;//- 0.035;
  //   // path_point.position.y = current_y + p_z.y ;//+ 0.080;
  //   // path_point.position.z = current_z + p_z.z ;//+ 0.036;

  //   path_point.position.x = p.x;//- 0.035;
  //   path_point.position.y = p.y;//+ 0.080;
  //   path_point.position.z = p.z;//+ 0.036;

  //   path_publisher.publish(path_point);

  //   cout << "i: " << i + 1 << endl;
  //   cout << "path_point: " << path_point << endl << endl;
  //   naptime.sleep(); // wait for remainder of specified period; 
  // }
  // cout << "3D path is published !!!!!!!!!" << endl;

}

int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::Rate naptime(1000); // use to regulate loop rate 

  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("robot_currentpose", 10, robot_currentpose_Callback);
 
  //publisher:
  ros::Publisher path_publisher       = nh.advertise<geometry_msgs::Pose>("motion_Path", 1);
  ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("generated_pc", 1);

  sensor_msgs::PointCloud2 pub_path;
  sensor_msgs::PointCloud2 pub_pointcloud;
 
  while (ros::ok()) 
  {

    if(receive_pose_flag == 1)
    {
      receive_pose_flag = 0;
      seam_detection(naptime, path_publisher, pub_pointcloud, pointcloud_publisher);
    }

    ros::spinOnce(); //allow data update from callback; 
    naptime.sleep(); // wait for remainder of specified period; 
  }


  ros::spin();
 
}
