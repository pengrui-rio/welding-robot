#include <ros/ros.h>
#include <math.h>
#include <iostream>   

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

// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud; 
typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;
typedef pcl::PointXYZ PointType;

using namespace cv;
using namespace std;
//namespace enc = sensor_msgs::image_encodings;

// 相机内参
const double camera_factor = 985;
const double camera_cx = 320;
const double camera_cy = 240;
const double camera_fx = 615.899;
const double camera_fy = 616.468;

// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;

//receive robot current pose flag
int receive_pose_flag = 0;
float current_x = 0  , current_y = 0    , current_z = 0;
float current_yaw = 0, current_pitch = 0, current_roll = 0;


void color_Callback(const sensor_msgs::ImageConstPtr& color_msg)
{
  try
  {
    cv::imshow("color_view", cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8)->image);
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

  if (current_roll != 0)
  {
    receive_pose_flag = 1;
  }
  cout << " \n" << endl; //add two more blank row so that we can see the message more clearly
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  //subscriber:
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
  image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image_rect_raw", 1, depth_Callback);
  ros::Subscriber sub = nh.subscribe("robot_currentpose", 10, robot_currentpose_Callback);
 
  //publisher:
  ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("generated_pc", 1);
  ros::Publisher map_publisher = nh.advertise<sensor_msgs::PointCloud2>("generated_map", 1);

  // 点云变量
  // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
  PointCloud::Ptr cloud ( new PointCloud );
  PointCloud::Ptr map   ( new PointCloud );

  sensor_msgs::PointCloud2 pub_pointcloud;
  sensor_msgs::PointCloud2 pub_map;

  pcl::PCDWriter writer;

  double sample_rate = 1000.0; // 1000HZ 
  ros::Rate naptime(sample_rate); // use to regulate loop rate 
 
  int initial_flag = 1;

  float pic_count = 0;


  



  //seam detection
  Cloud::Ptr cloud_ptr (new Cloud);
  PointCloud::Ptr cloud_output (new PointCloud);

  pcl::PCDReader reader;
  reader.read("/home/rick/Documents/a_system/src/seam_detection/save_pcd/test.pcd", *cloud_ptr);
  
  cout << "PointCLoud size() " << cloud_ptr->width * cloud_ptr->height
       << " data points " << pcl::getFieldsList (*cloud_ptr) << "." << endl;

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>& cloud_normals = *cloud_normals_ptr;
  ne.setRadiusSearch (0.005);
  ne.compute (cloud_normals);

  float nan_count = 0;
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  { 
    if ( __isnan(cloud_normals[i].curvature) == true)
    {
      continue;
    }

    // cout << "cloud_normal XYZ-Curvature: " << cloud_normals[i] << endl;

    pcl::PointXYZRGB p;

    p.x = cloud_ptr->points[i].x; 
    p.y = cloud_ptr->points[i].y;
    p.z = cloud_ptr->points[i].z;
    p.b = 200; 
    p.g = 200; 
    p.r = 200; 

    if(cloud_normals[i].normal_z < -0.9)
    {
      p.b = 200; 
      p.g = 0; 
      p.r = 0; 
    }
    
    cloud_output->points.push_back( p );    
  }

  cout << "compute is done!!! " << endl;
  cout << "cloud_normals size():" << cloud_normals.size() << endl;
  cout << "cloud_ptr->points.size(): " << cloud_output->points.size() << endl;
  cout << "nan-point count: " << cloud_normals.size() - cloud_output->points.size() << endl;
















  while (ros::ok()) 
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
        p.z = double(d) / camera_factor;
        p.x = (n - camera_cx) * p.z / camera_fx; 
        p.y = (m - camera_cy) * p.z / camera_fy;

        // 从rgb图像中获取它的颜色
        // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
        p.b = color_pic.ptr<uchar>(m)[n*3];
        p.g = color_pic.ptr<uchar>(m)[n*3+1];
        p.r = color_pic.ptr<uchar>(m)[n*3+2];

        // cout << "p.b:" << p.b << endl;
        // 把p加入到点云中
        cloud->points.push_back( p );        
      }
    }
 
    // pic_count++;
    // cout << "pic_count :" << pic_count << endl;
    // if (pic_count >= 2000 && initial_flag == 1)
    // {
      
    //   cloud->width = 1;
    //   cloud->height = cloud->points.size();

    //   cout << "cloud->points.size()" << cloud->points.size() << endl;
    //   pcl::PCDWriter writer;
    //   writer.write("/home/rick/Documents/a_system/src/seam_detection/save_pcd/test.pcd", *cloud, false) ;

    //   initial_flag = 0;
    // }
    // pcl::toROSMsg(*cloud, pub_pointcloud);


    pcl::toROSMsg(*cloud_output, pub_pointcloud);
    // pcl::toROSMsg(*map  , pub_map);

    pub_pointcloud.header.frame_id = "camera_color_optical_frame";
    pub_pointcloud.header.stamp = ros::Time::now();

    pub_map.header.frame_id = "world";
    pub_map.header.stamp = ros::Time::now();

    pointcloud_publisher.publish(pub_pointcloud);
    // map_publisher.publish(pub_map);
 
    cloud->points.clear();
    // map->points.clear();

    ros::spinOnce(); //allow data update from callback; 
    naptime.sleep(); // wait for remainder of specified period; 
  }


  ros::spin();
 
}
