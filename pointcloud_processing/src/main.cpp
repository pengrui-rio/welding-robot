#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>
#include <seam_location.h>
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
const double camera_cx = 311.2325744628906;
const double camera_cy = 226.9261474609375;
const double camera_fx = 619.9661254882812;
const double camera_fy = 619.856201171875;

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
  cv::imshow("color_view", color_pic);
  // RGBimage_seam_extration(color_pic);

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
  RGBimage_seam_extration(color_pic, depth_ptr->image);

  float xMin = 180, xMax = 530;
  float yMin = 140, yMax = 300;

  depth_pic(cv::Rect(xMin,yMin,xMax-xMin,yMax-yMin)).copyTo(depth_pic);


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
  ros::init(argc, argv, "seam_detection");
  ros::NodeHandle nh;
  ros::Rate naptime(10); // use to regulate loop rate 

  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("robot_currentpose", 10, robot_currentpose_Callback);
  image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
  image_transport::Subscriber depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_Callback);///camera/depth/image_rect_raw
 
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
  Cloud::Ptr      cloud_ptr         (new Cloud);


  seam_detection(naptime, cloud_ptr, path_publisher, pub_pointcloud, pointcloud_publisher);


  while (ros::ok()) 
  {
    // analyze_realsense_data(camera_pointcloud);
    // coordinate_transformation(camera_pointcloud, map_pointcloud, cloud_ptr);

    // pcl::toROSMsg(*camera_pointcloud, pub_camera_pointcloud);
    // pub_camera_pointcloud.header.frame_id = "camera_color_optical_frame";
    // pub_camera_pointcloud.header.stamp = ros::Time::now();
    // pointcloud_publisher.publish(pub_camera_pointcloud);


    // if(process_flag == 1)
    // {
    //   seam_detection(naptime, cloud_ptr, path_publisher, pub_pointcloud, pointcloud_publisher);
    //   break;
    // }

    camera_pointcloud->points.clear();
    map_pointcloud->points.clear();
    cloud_ptr->points.clear();

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
      p.z = double(d) / camera_factor;
      p.x = (180 + n - camera_cx) * p.z / camera_fx; 
      p.y = (140 + m - camera_cy) * p.z / camera_fy;

      // 从rgb图像中获取它的颜色
      // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
      p.b = 200;//color_pic.ptr<uchar>(m)[n*3];
      p.g = 200;//color_pic.ptr<uchar>(m)[n*3+1];
      p.r = 200;//color_pic.ptr<uchar>(m)[n*3+2];

      if( m == 80 || m == 72)
      {
        p.b = 0;
        p.g = 0;
        p.r = 200;
      }

      cloud->points.push_back( p );        
    }
  }
}

void coordinate_transformation(PointCloud::Ptr camera_pointcloud, PointCloud::Ptr map_pointcloud, Cloud::Ptr cloud_ptr)
{
  for (float i = 0; i < camera_pointcloud->points.size(); i++)  //480
  {
    pcl::PointXYZRGB p, p_z, p_x, p_y, p_pushback; 
    pcl::PointXYZ p_cloud_ptr;

    p.x    = camera_pointcloud->points[i].x; 
    p.y    = camera_pointcloud->points[i].y; 
    p.z    = camera_pointcloud->points[i].z; 

    rotate_y(p.x,   p.y,   p.z,   current_yaw    , &p_y.x, &p_y.y, &p_y.z);
    rotate_z(p_y.x, p_y.y, p_y.z, current_roll  , &p_x.x, &p_x.y, &p_x.z); 
    rotate_x(p_x.x, p_x.y, p_x.z, current_pitch  , &p_z.x, &p_z.y, &p_z.z);

    //bottom_straight:
    float l = 0.195, w = 0.195, o = 0.0;
    
    p_cloud_ptr.x = p_pushback.x = current_x + p_z.x + 0 + o ;//+ -0.034; 
    p_cloud_ptr.y = p_pushback.y = current_y + p_z.y + 0 + w;// + 0.1765; 
    p_cloud_ptr.z = p_pushback.z = current_z + p_z.z + 0 + l  ;//+ 0.225; 

    p_pushback.b = camera_pointcloud->points[i].b;
    p_pushback.g = camera_pointcloud->points[i].g;
    p_pushback.r = camera_pointcloud->points[i].r;

    map_pointcloud->points.push_back( p_pushback );    

    cloud_ptr->points.push_back( p_cloud_ptr );
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
  cloud->points.clear();
  show_Rviz_cloud->points.clear();
}


void seam_detection(ros::Rate naptime, Cloud::Ptr cloud_ptr, ros::Publisher path_publisher, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher)
{
  clock_t begin = clock();


  int show_Pointcloud_timeMax = 100;

  float sphere_computation = 0.005;


  cout << "1.读入原始pointcloud" << endl;
  PointCloud::Ptr cloud_ptr_show (new PointCloud);
  Cloud::Ptr cloud_ptr_new = read_pointcloud(cloud_ptr_show);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  ////////////////////////////////////////////////////////////

  cout << "2.算出所有点的法向量" << endl;
  PointNormal_Computation(cloud_ptr_new, cloud_ptr_show);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);






















  ////////////////////////////////////////////////////////////

  // //2.算出所有点的法向量
  // vector<Point3f> cloud_normals = allPoint_normal_computation(sphere_computation, cloud_ptr);
  // ////////////////////////////////////////////////////////////

  // //3.计算基准法向量，并返回提出nan点后的点云
  // float basic_normal_x = 0, basic_normal_y = 0, basic_normal_z = 0;
  // basic_normal_computation(cloud_ptr, cloud_normals, &basic_normal_x, &basic_normal_y, &basic_normal_z );
  // ////////////////////////////////////////////////////////////

  // //4.计算每个点的方向描述子
  // PointCloud::Ptr descriptor_cloud (new PointCloud);
  // vector<float> Dir_descriptor = Point_descriptor_computation(descriptor_cloud, cloud_ptr, cloud_normals, basic_normal_x, basic_normal_y, basic_normal_z);
  // show_pointcloud_Rviz(show_Pointcloud_timeMax, descriptor_cloud, pub_pointcloud, pointcloud_publisher);
  // ////////////////////////////////////////////////////////////

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
  // vector<float> orientation_pathpoints = Path_Generation(seg_pointcloud, cloud_seamRegion, path_cloud, path_cloud_showRviz);
  // show_pointcloud_Rviz(100*show_Pointcloud_timeMax, cloud_seamRegion, pub_pointcloud, pointcloud_publisher);
  // show_pointcloud_Rviz(100*show_Pointcloud_timeMax, path_cloud_showRviz, pub_pointcloud, pointcloud_publisher);

  // PointCloud::Ptr grooveRegion_onProfile = show_grooveRegion_onProfile(cloud_ptr, path_cloud_showRviz);
  // show_pointcloud_Rviz(1*show_Pointcloud_timeMax, grooveRegion_onProfile, pub_pointcloud, pointcloud_publisher);
  // //////////////////////////////////////////////////////////
  // cout << "3D path is generated !!!!!!!!!" << endl;

  // geometry_msgs::Pose path_point;
  // for(int i = 0; i < path_cloud->points.size(); i++)
  // {
  //   path_point.position.x = path_cloud->points[i].x;//- 0.035;
  //   path_point.position.y = path_cloud->points[i].y ;
  //   path_point.position.z = path_cloud->points[i].z;//+ 0.036;
  //   path_point.orientation.x = orientation_pathpoints[i];

  //   path_publisher.publish(path_point);

  //   naptime.sleep(); // wait for remainder of specified period; 
  // }
  // cout << "3D path is published !!!!!!!!!" << endl;


  clock_t end = clock();
  double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
  cout << elapsed_secs << " s" << endl;
    
}
