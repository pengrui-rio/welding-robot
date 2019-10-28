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
#include <pcl/filters/voxel_grid.h>

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
const double camera_factor = 1000;
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

  receive_pose_flag = 1;

  cout << " \n" << endl; //add two more blank row so that we can see the message more clearly
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
      p.z = double(d) / camera_factor;
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
}

void map_reconstruction(PointCloud::Ptr camera_pointcloud, PointCloud::Ptr map_pointcloud)
{
  for (float i = 0; i < camera_pointcloud->points.size(); i++)  //480
  {
    pcl::PointXYZRGB p, p_z, p_x, p_y, p_pushback;

    p.x    = camera_pointcloud->points[i].x; 
    p.y    = camera_pointcloud->points[i].y; 
    p.z    = camera_pointcloud->points[i].z; 

    rotate_y(p.x,   p.y,   p.z,   current_roll  , &p_y.x, &p_y.y, &p_y.z);
    rotate_x(p_y.x, p_y.y, p_y.z, current_pitch , &p_x.x, &p_x.y, &p_x.z);
    rotate_z(p_x.x, p_x.y, p_x.z, current_yaw   , &p_z.x, &p_z.y, &p_z.z); 

    p_pushback.x = current_x + p_z.x ;//- 0.035;
    p_pushback.y = current_y + p_z.y ;//+ 0.080;
    p_pushback.z = current_z + p_z.z ;//+ 0.036;
    p_pushback.b = camera_pointcloud->points[i].b;
    p_pushback.g = camera_pointcloud->points[i].g;
    p_pushback.r = camera_pointcloud->points[i].r;

    map_pointcloud->points.push_back( p_pushback );     
  }

  // double gridsize = 0.001;
  // static pcl::VoxelGrid<PointT> voxel;
  // voxel.setLeafSize( gridsize, gridsize, gridsize );
  // voxel.setInputCloud( map_pointcloud );

  // PointCloud::Ptr tmp( new PointCloud() );
  // voxel.filter( *tmp );

}


void record_mapPointcloud( PointCloud::Ptr cloud )
{
  PointCloud::Ptr cloud_export (new PointCloud);

  for(float i = 0; i < cloud->points.size(); i++)
  {
    pcl::PointXYZRGB p;
    p.x = cloud->points[i].x; 
    p.y = cloud->points[i].y;
    p.z = cloud->points[i].z;
    p.r = cloud->points[i].r; 
    p.g = cloud->points[i].g;
    p.b = cloud->points[i].b;

    if ( (p.x <= 1 && p.x >= -1) && (p.y <= 1 && p.y >= -1) && (p.z <= 1 && p.z >= -1) )
    {
      cloud_export->points.push_back( p );    
    }
  }

  cloud_export->width = 1;
  cloud_export->height = cloud_export->points.size();

  cout << "cloud->points.size()" << cloud_export->points.size() << endl;
  pcl::PCDWriter writer;
  writer.write("./src/surface_reconstruction/save_pcd/map.pcd", *cloud_export, false) ;
  
}



void record_single_rgbdFrame(int initial_flag, float pic_count, PointCloud::Ptr cloud)
{
  cout << "pic_count :" << pic_count << endl;
  if (pic_count >= 1000 && initial_flag == 1)
  {

    PointCloud::Ptr cloud_export (new PointCloud);

    for(float i = 0; i < cloud->points.size(); i++)
    {
      pcl::PointXYZRGB p;
      p.x = cloud->points[i].x; 
      p.y = cloud->points[i].y;
      p.z = cloud->points[i].z;
      p.r = cloud->points[i].r; 
      p.g = cloud->points[i].g;
      p.b = cloud->points[i].b;

      if ( (p.x <= 1 && p.x >= -1) && (p.y <= 1 && p.y >= -1) && (p.z <= 1 && p.z >= -1) )
      {
        cloud_export->points.push_back( p );    
      }
    }

    cloud_export->width = 1;
    cloud_export->height = cloud_export->points.size();

    cout << "cloud->points.size()" << cloud_export->points.size() << endl;
    pcl::PCDWriter writer;
    writer.write("./src/seam_detection/save_pcd/ICP2.pcd", *cloud_export, false) ;

    initial_flag = 0;
  }
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
  ros::Publisher camera_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("camera_pointcloud", 1);
  ros::Publisher map_pointcloud_publisher    = nh.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 1);
 
  //pointcloud msgs: 
  sensor_msgs::PointCloud2 pub_camera_pointcloud;
  sensor_msgs::PointCloud2 pub_map_pointcloud;


  PointCloud::Ptr camera_pointcloud (new PointCloud);
  PointCloud::Ptr map_pointcloud    (new PointCloud);

  int receive_pose_count = 0;
  while (ros::ok()) 
  {
    analyze_realsense_data(camera_pointcloud);

    pcl::toROSMsg(*camera_pointcloud, pub_camera_pointcloud);
    pub_camera_pointcloud.header.frame_id = "camera_color_optical_frame";
    pub_camera_pointcloud.header.stamp = ros::Time::now();
    camera_pointcloud_publisher.publish(pub_camera_pointcloud);


    if(receive_pose_flag)
    {
      map_reconstruction(camera_pointcloud, map_pointcloud);
      receive_pose_flag = 0;
      receive_pose_count++;

      cout << "map_pointcloud->points.size()  " << map_pointcloud->points.size() << endl;
      if(receive_pose_count == 3)
      {
        record_mapPointcloud(map_pointcloud);
        cout << " record mapPointcloud !!!!" << endl ;
      }
    }
    
    pcl::toROSMsg(*map_pointcloud, pub_map_pointcloud);
    pub_map_pointcloud.header.frame_id = "base_link";
    pub_map_pointcloud.header.stamp = ros::Time::now();
    map_pointcloud_publisher.publish(pub_map_pointcloud);


    camera_pointcloud->points.clear();

    ros::spinOnce(); //allow data update from callback; 
    naptime.sleep(); // wait for remainder of specified period; 
  }


  ros::spin();
 
}
