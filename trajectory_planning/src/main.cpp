#include <ros/ros.h>
#include <math.h>
#include <iostream>   
#include <vector>
#include <seam_location.h>
#include <motion_planning.h>
#include <transformation.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
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
double camera_factor = 1000;
double camera_cx = 311.2325744628906;
double camera_cy = 226.9261474609375;
double camera_fx = 619.9661254882812;
double camera_fy = 619.856201171875;

// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;

//receive robot current pose flag
int receive_pose_flag = 0, process_flag = 0, process_count = 0, process_count_limit = 10;
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
  // cv::imshow("color_view", color_pic);
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
  // cv::imshow("depth_pic", depth_pic);


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

void trajectory_planning(Cloud::Ptr PathPoint_Position, ros::Rate naptime, Cloud::Ptr cloud_ptr, pcl::PointXYZ realsense_position, ros::Publisher path_publisher, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher);
void show_pointcloud_Rviz(int show_Pointcloud_timeMax, PointCloud::Ptr show_Rviz_cloud, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher);
void publish_pointcloud_Rviz(string coordinate, PointCloud::Ptr pointloud, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher);


int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "trajectory_planning");
  ros::NodeHandle nh;
  ros::Rate naptime(10); // use to regulate loop rate 

  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("robot_currentpose", 10, robot_currentpose_Callback);
  image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
  image_transport::Subscriber depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_Callback);///camera/depth/image_rect_raw
 
  //publisher:
  ros::Publisher path_publisher              = nh.advertise<geometry_msgs::Pose>("motion_Path", 1);
  ros::Publisher pointcloud_publisher        = nh.advertise<sensor_msgs::PointCloud2>("processing_pointcloud", 1);
  ros::Publisher camera_pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("camera_pointcloud", 1);
  ros::Publisher map_pointcloud_publisher    = nh.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 1);
  ros::Publisher vis_pub                     = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  //pointcloud msgs: 
  sensor_msgs::PointCloud2 pub_camera_pointcloud;
  sensor_msgs::PointCloud2 pub_map_pointcloud;
  sensor_msgs::PointCloud2 pub_path;
  sensor_msgs::PointCloud2 pub_pointcloud;
 
  //pointcloud:
  PointCloud::Ptr camera_pointcloud (new PointCloud);
  PointCloud::Ptr map_pointcloud    (new PointCloud);
  Cloud::Ptr      cloud_ptr         (new Cloud);   Cloud::Ptr  cloud_ptr_filter  (new Cloud); Cloud::Ptr  cloud_ptr_input  (new Cloud);
  Cloud::Ptr PathPoint_Position      (new Cloud);

  //tf:
  tf::TransformListener listener;
  tf::TransformBroadcaster tf_broadcaster;
 
  int send_flag = 0;
  while (ros::ok()) 
  {
    
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform("/base_link", "/tool0", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    pcl::PointXYZ realsense_position = realsense_position_acquisition(transform);
    cout << "realsense_position: " << endl << realsense_position << endl;
    //////////////////////////////////////////////////////////////////////////////////////

    analyze_realsense_data(camera_pointcloud);
    publish_pointcloud_Rviz("camera_color_optical_frame", camera_pointcloud, pub_camera_pointcloud, camera_pointcloud_publisher);
    //////////////////////////////////////////////////////////////////////////////////////

    coordinate_transformation(transform, camera_pointcloud, map_pointcloud, cloud_ptr);
    publish_pointcloud_Rviz("base_link", map_pointcloud, pub_map_pointcloud, map_pointcloud_publisher);
    //////////////////////////////////////////////////////////////////////////////////////



    // Eigen::Quaterniond q = Transform_AngleAxisd_Quatenion();
    // double px = 0.056;//double px = -0.04;
    // double py = 0.328;
    // double pz = 0.1;
    // float T_o_B_q[4];
    // double qx = q.x();
    // double qy = q.y();
    // double qz = q.z();
    // double qw = q.w();

    // tf::Quaternion rotation (qx,qy,qz,qw);
    // tf::Vector3 origin (px,py,pz);
    // tf::Transform t (rotation, origin);

    // std::string markerFrame = "GT_ar_marker_";
    // std::stringstream out;
    // out << 1;
    // std::string id_string = out.str();
    // markerFrame += id_string;
    // tf::StampedTransform GT_Marker (t, ros::Time::now(), "/base_link", markerFrame.c_str());
    // tf_broadcaster. sendTransform(GT_Marker);

    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "base_link";
    // marker.header.stamp = ros::Time();
    // marker.ns = "my_namespace";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::ARROW;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = 0;
    // marker.pose.position.y = 0.6;
    // marker.pose.position.z = 0.1;
    // marker.pose.orientation.x = q.x();
    // marker.pose.orientation.y = q.y();
    // marker.pose.orientation.z = q.z();
    // marker.pose.orientation.w = q.w();
    // marker.scale.x = 0.05;
    // marker.scale.y = 0.001;
    // marker.scale.z = 0.001;
    // marker.color.a = 1.0; // Don't forget to set the alpha!
    // marker.color.r = 1;
    // marker.color.g = 0;
    // marker.color.b = 0;
    // vis_pub.publish( marker );


    //////////////////////////////////////////////////////////////////////////////////////




    if(process_flag == 1)
    {
      process_count++;
      cloud_ptr_input = input_pointcloud_filter(process_count, process_count_limit, cloud_ptr, cloud_ptr_filter);
      cout << "process_count: " << process_count << " cloud_ptr_filter->points.size(): " << cloud_ptr_filter->points.size() << endl; 

      if(process_count >= process_count_limit)
      {
        send_flag = 1;
        process_flag = 0; //process_count = 0;
        cout << " cloud_ptr_input->points.size(): " << cloud_ptr_input->points.size() << endl; 
        cout << "trajectory_planning" << endl;
        trajectory_planning(PathPoint_Position, naptime, cloud_ptr_input, realsense_position, path_publisher, pub_pointcloud, pointcloud_publisher);
        // break;
      }
    }

    if(process_count >= process_count_limit)
    {
      vector<Point3f> move_Vector;
      for(float i = 0; i < PathPoint_Position->points.size() - 1; i++)
      {
        Point3f v;
        if(PathPoint_Position->points[i].x < PathPoint_Position->points[i+1].x)
        {
          v.x = PathPoint_Position->points[i+1].x - PathPoint_Position->points[i].x;
          v.y = PathPoint_Position->points[i+1].y - PathPoint_Position->points[i].y;
        }
        else
        {
          v.x = PathPoint_Position->points[i].x - PathPoint_Position->points[i+1].x;
          v.y = PathPoint_Position->points[i].y - PathPoint_Position->points[i+1].y;
        }
        move_Vector.push_back(v);
      }
      cout << "move_Vector.size()" << move_Vector.size() << endl;

      vector <double> yaw_vec;
      for(float i = 0; i < move_Vector.size() - 1; i++)
      {
        double x = move_Vector[i].x, y = move_Vector[i].y;//, z = -1.0/sqrt(2);
        double pitch = 0, roll = 0, yaw = 0;

        //yaw:
        if(y > 0 && x > 0)
          yaw = 0   + atan2(abs(y), abs(x)) * 180 / M_PI;
        if(y > 0 && x < 0)
          yaw = 90  + atan2(abs(x), abs(y)) * 180 / M_PI;
        if(y < 0 && x < 0)
          yaw = 180 + atan2(abs(y), abs(x)) * 180 / M_PI;
        if(y < 0 && x > 0)
          yaw = 270 + atan2(abs(x), abs(y)) * 180 / M_PI;

        float q[4];
        euler_to_quaternion(yaw, pitch, roll, q);

        double px = PathPoint_Position->points[i].x;//double px = -0.04;
        double py = PathPoint_Position->points[i].y;
        double pz = PathPoint_Position->points[i].z;
        // float T_o_B_q[4];
        double qx = q[0];
        double qy = q[1];
        double qz = q[2];
        double qw = q[3];

        tf::Quaternion rotation (qx,qy,qz,qw);
        tf::Vector3 origin (px,py,pz);
        tf::Transform t (rotation, origin);

        std::string markerFrame = "GT_ar_marker_";
        std::stringstream out;
        out << i;
        std::string id_string = out.str();
        markerFrame += id_string;
        tf::StampedTransform GT_Marker (t, ros::Time::now(), "/base_link", markerFrame.c_str());
        tf_broadcaster.sendTransform(GT_Marker);
        cout<< "marker: " << i << endl;

        yaw_vec.push_back(yaw);
      }
      // break;


      if(send_flag == 1)
      {
        send_flag =  0;
        geometry_msgs::Pose path_point;
        for(int i = 0; i < PathPoint_Position->points.size(); i++)
        {
          path_point.position.x = PathPoint_Position->points[i].x;//- 0.035;
          path_point.position.y = PathPoint_Position->points[i].y ;
          path_point.position.z = PathPoint_Position->points[i].z;//+ 0.036;


          if(i == yaw_vec.size() - 1)
            break;
            
          float q[4];
          euler_to_quaternion(yaw_vec[i], 225, 0, q) ;
          path_point.orientation.x = q[0];
          path_point.orientation.y = q[1];
          path_point.orientation.z = q[2];
          path_point.orientation.w = q[3];

          path_publisher.publish(path_point);

          naptime.sleep(); // wait for remainder of specified period; 
        }
        cout << "3D path is published !!!!!!!!!" << endl;
      }
      yaw_vec.clear();
    }
  



    //////////////////////////////////////////////////////////////////////////////////////
    camera_pointcloud->points.clear();
    map_pointcloud->points.clear();
    cloud_ptr->points.clear();

    ros::spinOnce(); //allow data update from callback; 
    // naptime.sleep(); // wait for remainder of specified period; 
  }


  // ros::spin();

  return 0;
}


void trajectory_planning(Cloud::Ptr PathPoint_Position, ros::Rate naptime, Cloud::Ptr cloud_ptr, pcl::PointXYZ realsense_position, ros::Publisher path_publisher, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher)
{
  clock_t begin = clock();

  int show_Pointcloud_timeMax = 100;
  float sphere_computation = 0.005;

  //Seam Location:    
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "1.读入原始pointcloud" << endl << endl;
  PointCloud::Ptr cloud_ptr_show (new PointCloud);
  // Cloud::Ptr cloud_ptr_new    = read_pointcloud(cloud_ptr_show);
  SurfaceProfile_Reconstruction(cloud_ptr, cloud_ptr_show);
  Cloud::Ptr cloud_ptr_origin = cloud_ptr_origin_copy(cloud_ptr);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "2.算出所有点的法向量" << endl << endl;
  Point3f Cam_Position; Cam_Position.x = realsense_position.x; Cam_Position.y = realsense_position.y; Cam_Position.z = realsense_position.z; 
  vector<Point3f> Normal = PointNormal_Computation(cloud_ptr, cloud_ptr_show, Cam_Position);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "3.剔除均匀变化的部分" << endl << endl;
  Delete_SmoothChange_Plane(cloud_ptr, cloud_ptr_show, Normal, pub_pointcloud, pointcloud_publisher);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "4.人工筛选出可能的焊接缝" << endl << endl;
  int seam_label = 0;
  Screen_Candidate_Seam(cloud_ptr, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);  // geometry_msgs::Pose path_point;
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);

  //Trajectory Planning:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "5.提取焊接缝边界" << endl << endl;
  Cloud::Ptr seam_edge = Extract_Seam_edge(cloud_ptr, cloud_ptr_show);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "6.焊接缝轨迹生成" << endl << endl; 
  Cloud::Ptr PathPoint_Position1 = PathPoint_Position_Generation(seam_edge, cloud_ptr_origin, cloud_ptr_show);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "7.焊接缝轨迹点旋转方向" << endl << endl;
  vector<Point3f> Normal_Vector = PathPoint_Orientation_Generation(PathPoint_Position1, cloud_ptr_origin, cloud_ptr, cloud_ptr_show);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);

  for(int i = 0; i < PathPoint_Position1->points.size(); i++)
  {
    pcl::PointXYZ p;
    p.x = PathPoint_Position1->points[ i ].x;
    p.y = PathPoint_Position1->points[ i ].y;
    p.z = PathPoint_Position1->points[ i ].z;

    PathPoint_Position->points.push_back( p );  
  }


  // geometry_msgs::Pose path_point;
  // for(int i = 0; i < PathPoint_Position->points.size(); i++)
  // {
  //   path_point.position.x = PathPoint_Position->points[i].x;//- 0.035;
  //   path_point.position.y = PathPoint_Position->points[i].y ;
  //   path_point.position.z = PathPoint_Position->points[i].z;//+ 0.036;
  //   // path_point.orientation.x = orientation_pathpoints[i];

  //   path_publisher.publish(path_point);

  //   naptime.sleep(); // wait for remainder of specified period; 
  // }
  // cout << "3D path is published !!!!!!!!!" << endl;

  cout << endl;
  clock_t end = clock();
  double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
  cout << elapsed_secs << " s" << endl;
    
  // return PathPoint_Position;
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void show_pointcloud_Rviz(int show_Pointcloud_timeMax, PointCloud::Ptr show_Rviz_cloud, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher)
{
  for(float i = 0; i < show_Pointcloud_timeMax; i++)
  {
    pcl::toROSMsg(*show_Rviz_cloud, pub_pointcloud);
    pub_pointcloud.header.frame_id = "base_link";
    pub_pointcloud.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(pub_pointcloud);
  }
  show_Rviz_cloud->points.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void publish_pointcloud_Rviz(string coordinate, PointCloud::Ptr pointloud, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher)
{
    pcl::toROSMsg(*pointloud, pub_pointcloud);
    pub_pointcloud.header.frame_id = coordinate;// 
    pub_pointcloud.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(pub_pointcloud);
}
