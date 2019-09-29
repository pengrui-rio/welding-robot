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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/registration/icp.h>

#include <transformation.h>

// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB>  PointCloud; 
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloudL;  
typedef pcl::PointCloud<pcl::PointXYZ>     Cloud;
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

Cloud::Ptr read_pointcloud (PointCloud::Ptr cloud_ptr_show)
{
  //seam detection
  Cloud::Ptr cloud_ptr (new Cloud);

  pcl::PCDReader reader;
  reader.read("/home/rick/Documents/a_system/src/seam_detection/save_pcd/run.pcd", *cloud_ptr);
  
  cout << "PointCLoud size() " << cloud_ptr->width * cloud_ptr->height
       << " data points " << pcl::getFieldsList (*cloud_ptr) << "." << endl << endl;

  for(float i = 0; i < cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZRGB p;
    p.x = cloud_ptr->points[i].x; 
    p.y = cloud_ptr->points[i].y;
    p.z = cloud_ptr->points[i].z;
    p.b = 200; 
    p.g = 200;
    p.r = 200;
    cloud_ptr_show->points.push_back( p );    
  }

  return cloud_ptr;
}

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  ros::Rate naptime(1000.0); // use to regulate loop rate 

  //subscriber:
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
  image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image_rect_raw", 1, depth_Callback);
  ros::Subscriber sub = nh.subscribe("robot_currentpose", 10, robot_currentpose_Callback);
 
  //publisher:
  ros::Publisher intial_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("intial_pointcloud", 1);
  ros::Publisher icp_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("icp_pointcloud", 1);
 
  //pointcloud msgs: 
  sensor_msgs::PointCloud2 pub_intial_pointcloud;
  sensor_msgs::PointCloud2 pub_icp_pointcloud;



  //////////////////////////////////////////////////////////////////////



 
  // Cloud::Ptr cloud_ptr (new Cloud);
  // Cloud::Ptr cloud_export (new Cloud);

  // pcl::PCDReader reader;                                                                       
  // reader.read("/home/rick/Documents/a_system/src/seam_detection/save_pcd/run.pcd", *cloud_ptr);
  
  // cout << "PointCLoud size() " << cloud_ptr->width * cloud_ptr->height
  //      << " data points " << pcl::getFieldsList (*cloud_ptr) << "." << endl << endl;
 
  // for(float i = 0; i < cloud_ptr->points.size(); i++)
  // {
  //   pcl::PointXYZ p;
  //   p.x = cloud_ptr->points[i].x; 
  //   p.y = cloud_ptr->points[i].y;
  //   p.z = cloud_ptr->points[i].z;

  //   if ( (p.x <= 1 && p.x >= -1) && (p.y <= 1 && p.y >= -1) && (p.z <= 1 && p.z >= -1) )
  //   {
  //     cloud_export->points.push_back( p );    
  //   }
  // }

  // cloud_export->width = 1;
  // cloud_export->height = cloud_export->points.size();

  // cout << "cloud->points.size()" << cloud_export->points.size() << endl;
  // pcl::PCDWriter writer;
  // writer.write("/home/rick/Documents/a_system/src/seam_detection/save_pcd/run_export.pcd", *cloud_export, false) ;




  int iterations = 1;  // Default number of ICP iterations

    // The point clouds we will be using
  Cloud::Ptr cloud_icp1_ptr  (new Cloud);  // Original point cloud
  Cloud::Ptr cloud_icp2_ptr (new Cloud);  // ICP output point cloud

  PointCloud::Ptr cloud_icp1 (new PointCloud);  // ICP output point cloud
  PointCloud::Ptr cloud_icp2 (new PointCloud);  // ICP output point cloud
  PointCloud::Ptr cloud_icp1_show (new PointCloud);  // ICP output point cloud

  // count the time
  pcl::console::TicToc time;
  pcl::PCDReader reader;

  // read the point cloud
  time.tic ();
  reader.read("./src/seam_detection/save_pcd/ICP1.pcd", *cloud_icp1);
  cout << "\nLoaded file " << " (" << cloud_icp1->points.size() << " points) in " << time.toc () << " ms\n" << std::endl;

  for(float i = 0; i < cloud_icp1->points.size(); i++)
  {
    pcl::PointXYZ p;
    p.x = cloud_icp1->points[i].x; 
    p.y = cloud_icp1->points[i].y;
    p.z = cloud_icp1->points[i].z;

    cloud_icp1_ptr->points.push_back( p );    
  }

  pcl::toROSMsg(*cloud_icp1, pub_intial_pointcloud);
  pub_intial_pointcloud.header.frame_id = "camera_color_optical_frame";
  pub_intial_pointcloud.header.stamp = ros::Time::now();
  intial_pointcloud_publisher.publish(pub_intial_pointcloud);
  // cloud_icp1->points.clear();

  time.tic ();
  reader.read("./src/seam_detection/save_pcd/ICP2.pcd", *cloud_icp2);
  cout << "\nLoaded file " << " (" << cloud_icp2->points.size() << " points) in " << time.toc () << " ms\n" << std::endl;

  for(float i = 0; i < cloud_icp2->points.size(); i++)
  {
    pcl::PointXYZ p;
    p.x = cloud_icp2->points[i].x; 
    p.y = cloud_icp2->points[i].y;
    p.z = cloud_icp2->points[i].z;

    cloud_icp2_ptr->points.push_back( p );    
  }

  pcl::toROSMsg(*cloud_icp2, pub_icp_pointcloud);
  pub_icp_pointcloud.header.frame_id = "camera_color_optical_frame";
  pub_icp_pointcloud.header.stamp = ros::Time::now();
  icp_pointcloud_publisher.publish(pub_icp_pointcloud);
  // cloud_icp2->points.clear();

  // // Defining a rotation matrix and translation vector
  // Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  // // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  // double theta = M_PI / 12;  // The angle of rotation in radians
  // transformation_matrix (0, 0) = cos (theta);
  // transformation_matrix (0, 1) = -sin (theta);
  // transformation_matrix (1, 0) = sin (theta);
  // transformation_matrix (1, 1) = cos (theta);

  // // A translation on Z axis (0.4 meters)
  // transformation_matrix (0, 3) = 0.1;
  // transformation_matrix (1, 3) = 0.1;
  // transformation_matrix (2, 3) = 0.1;

  // // Display in terminal the transformation matrix
  // cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << endl;
  // print4x4Matrix (transformation_matrix);

  // // Executing the transformation
  // pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
  // // *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use


  // The Iterative Closest Point algorithm
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(0.1);  
  icp.setTransformationEpsilon(1e-10);  
  icp.setEuclideanFitnessEpsilon(0.01);  
  icp.setMaximumIterations (1);
  icp.setInputSource (cloud_icp1_ptr);
  icp.setInputTarget (cloud_icp2_ptr);


  while (ros::ok()) 
  {
    time.tic ();
    icp.align (*cloud_icp1_ptr);
    cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << endl;
    iterations++;

    for(float i = 0; i < cloud_icp1_ptr->points.size(); i++)
    {
      pcl::PointXYZRGB p;
      p.x = cloud_icp1_ptr->points[i].x; 
      p.y = cloud_icp1_ptr->points[i].y;
      p.z = cloud_icp1_ptr->points[i].z;
      p.b = cloud_icp1->points[i].b; 
      p.g = cloud_icp1->points[i].g;
      p.r = cloud_icp1->points[i].r;
      cloud_icp1_show->points.push_back( p );   
    }

    // for(float i = 0; i < cloud_icp2_ptr->points.size(); i++)
    // {
    //   pcl::PointXYZRGB p;
    //   p.x = cloud_icp->points[i].x; 
    //   p.y = cloud_icp->points[i].y;
    //   p.z = cloud_icp->points[i].z;
    //   p.b = 200; 
    //   p.g = 0;
    //   p.r = 0;
    //   cloud_icp_show->points.push_back( p );   
    // }

    if (icp.hasConverged ())
    {
      cout << "\nICP has converged, score is " << icp.getFitnessScore () << endl << endl;
      // cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << endl;
      // transformation_matrix = icp.getFinalTransformation ().cast<double>();
      // print4x4Matrix (transformation_matrix);
    }
    else
    {
      PCL_ERROR ("\nICP has not converged.\n");
      return (-1);
    }

    pcl::toROSMsg(*cloud_icp1_show, pub_intial_pointcloud);
    pub_intial_pointcloud.header.frame_id = "camera_color_optical_frame";
    pub_intial_pointcloud.header.stamp = ros::Time::now();
    intial_pointcloud_publisher.publish(pub_intial_pointcloud);
    cloud_icp1_show->points.clear();

    // pcl::toROSMsg(*cloud_icp2, pub_icp_pointcloud);
    // pub_icp_pointcloud.header.frame_id = "camera_color_optical_frame";
    // pub_icp_pointcloud.header.stamp = ros::Time::now();
    // icp_pointcloud_publisher.publish(pub_icp_pointcloud);
    // // cloud_icp_show->points.clear();

     
    ros::spinOnce(); //allow data update from callback; 
    naptime.sleep(); // wait for remainder of specified period; 
  }


  ros::spin();
 
}
