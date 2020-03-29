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
int receive_pose_flag = 0, trajectoryPlanning_flag = 0, process_count = 0, process_count_limit = 1;
float current_x = 0  , current_y = 0    , current_z = 0;
float current_yaw = 0, current_pitch = 0, current_roll = 0;

//pose of trajectory:
vector< geometry_msgs::Pose > Rviz_TrajectoryPose;

//marker:
vector< tf::StampedTransform > GT_Marker_all;

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
  // ROS_INFO("the position(x,y,z) is %f , %f, %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  // ROS_INFO("the orientation(yaw, pitch, roll) is %f , %f, %f ", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  // ROS_INFO("the time we get the pose is %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec);

  current_x     = msg->pose.position.x;
  current_y     = msg->pose.position.y; 
  current_z     = msg->pose.position.z;

  current_yaw   = msg->pose.orientation.x;
  current_pitch = msg->pose.orientation.y;
  current_roll  = msg->pose.orientation.z;

  trajectoryPlanning_flag  = msg->pose.orientation.w;
  receive_pose_flag = msg->pose.orientation.w;
  
  cout << " \n" << endl; //add two more blank row so that we can see the message more clearly
}
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////

vector< geometry_msgs::Pose > trajectory_planning(vector< geometry_msgs::Pose > &Welding_Trajectory,
                                                  Cloud::Ptr cloud_ptr1, 
                                                  pcl::PointXYZ realsense_position, 
                                                  sensor_msgs::PointCloud2 pub_pointcloud, 
                                                  ros::Publisher pointcloud_publisher);

void show_pointcloud_Rviz(int show_Pointcloud_timeMax, 
                          PointCloud::Ptr show_Rviz_cloud, 
                          sensor_msgs::PointCloud2 pub_pointcloud, 
                          ros::Publisher pointcloud_publisher);

void publish_pointcloud_Rviz(string coordinate, 
                        PointCloud::Ptr pointloud, 
                        sensor_msgs::PointCloud2 pub_pointcloud, 
                        ros::Publisher pointcloud_publisher);

vector< geometry_msgs::Pose > trajectory_6DOF_generation(int &trajectoryPlanning_flag, 
                                                         int &process_count, 
                                                         int process_count_limit, 
                                                         Cloud::Ptr cloud_ptr, 
                                                         Cloud::Ptr cloud_ptr_filter, 
                                                         pcl::PointXYZ realsense_position, 
                                                         ros::Rate naptime, 
                                                         sensor_msgs::PointCloud2 pub_pointcloud, 
                                                         ros::Publisher pointcloud_publisher, 
                                                         ros::Publisher Welding_Trajectory_publisher);

int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "trajectory_planning");
  ros::NodeHandle nh;
  ros::Rate naptime(10000); // use to regulate loop rate 

  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("robot_currentpose", 10, robot_currentpose_Callback);
  image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
  image_transport::Subscriber depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_Callback);///camera/depth/image_rect_raw
 
  //publisher:
  ros::Publisher Moveit_path_publisher        = nh.advertise<geometry_msgs::Pose>("Moveit_motion_Path", 1);
  ros::Publisher Welding_Trajectory_publisher = nh.advertise<geometry_msgs::Pose>("Welding_Trajectory", 1);
  ros::Publisher pointcloud_publisher         = nh.advertise<sensor_msgs::PointCloud2>("processing_pointcloud", 1);
  ros::Publisher camera_pointcloud_publisher  = nh.advertise<sensor_msgs::PointCloud2>("camera_pointcloud", 1);
  ros::Publisher cam_pc_transform_publisher   = nh.advertise<sensor_msgs::PointCloud2>("camera_pointcloud_transform", 1);
  ros::Publisher map_pointcloud_publisher     = nh.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 1);
  ros::Publisher vis_pub                      = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  //pointcloud msgs: 
  sensor_msgs::PointCloud2 pub_camera_pointcloud;
  sensor_msgs::PointCloud2 pub_cam_pc_transform;
  sensor_msgs::PointCloud2 pub_map_pointcloud;
  sensor_msgs::PointCloud2 pub_path;
  sensor_msgs::PointCloud2 pub_pointcloud;
 
  //pointcloud:
  PointCloud::Ptr camera_pointcloud (new PointCloud);
  PointCloud::Ptr cam_pc_transform  (new PointCloud);
  PointCloud::Ptr map_pointcloud    (new PointCloud);
  Cloud::Ptr      cloud_ptr         (new Cloud);   
  Cloud::Ptr      cloud_ptr_filter  (new Cloud); 
  Cloud::Ptr      cloud_ptr_input   (new Cloud);

  //tf:
  tf::TransformListener listener;
  tf::TransformBroadcaster tf_broadcaster;



  ///////////////////////////////////////////////////////////////////////////////
  while (ros::ok()) 
  {
    // break;
    tf::StampedTransform transform_baselink2tool0;
    tf::StampedTransform transform_tool02torch;
    try
    {
      listener.lookupTransform("/base_link", "/tool0", ros::Time(0)        , transform_baselink2tool0);
      listener.lookupTransform("/tool0"    , "/welding_torch", ros::Time(0), transform_tool02torch);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    //////////////////////////////////////////////////////////////////////////////////////
    Base_to_End_transform(receive_pose_flag, transform_baselink2tool0);

    geometry_msgs::Pose End_Torch = Torch_to_End_transform(transform_tool02torch);

    //////////////////////////////////////////////////////////////////////////////////////

    analyze_realsense_data(camera_pointcloud);
    publish_pointcloud_Rviz("camera_color_optical_frame",
                             camera_pointcloud, 
                             pub_camera_pointcloud, 
                             camera_pointcloud_publisher);
                             
    //////////////////////////////////////////////////////////////////////////////////////

    coordinate_transformation(transform_baselink2tool0, 
                              camera_pointcloud, 
                              cam_pc_transform, 
                              cloud_ptr);
    publish_pointcloud_Rviz("base_link", 
                             cam_pc_transform, 
                             pub_cam_pc_transform, 
                             cam_pc_transform_publisher);

    //////////////////////////////////////////////////////////////////////////////////////

    model_3D_reconstruction(receive_pose_flag, cam_pc_transform, map_pointcloud);
    publish_pointcloud_Rviz("base_link", 
                             map_pointcloud, 
                             pub_map_pointcloud, 
                             map_pointcloud_publisher);

    //////////////////////////////////////////////////////////////////////////////////////

    //在Rviz里面显示每display_pointsize个点的pose
    for(int i = 0; i < Rviz_TrajectoryPose.size(); i++)
    {
      int display_pointsize = 20;
      if( (Rviz_TrajectoryPose.size() / display_pointsize) != 0 && i%(Rviz_TrajectoryPose.size() / display_pointsize) == 0 )
      {
        std::string   markerFrame       = Waypoint_markerName_creation(i);
        tf::Transform waypoint_tranform = Waypoint_markerTransform_creation(i, Rviz_TrajectoryPose[i]);

        tf::StampedTransform waypoint_Marker (waypoint_tranform, ros::Time::now(), "/base_link", markerFrame.c_str());
        tf_broadcaster.sendTransform(waypoint_Marker);
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////


    Rviz_TrajectoryPose = trajectory_6DOF_generation( trajectoryPlanning_flag, 
                                                      process_count, 
                                                      process_count_limit, 
                                                      cloud_ptr, 
                                                      cloud_ptr_filter, 
                                                      realsense_position_acquisition(transform_baselink2tool0), 
                                                      naptime, 
                                                      pub_pointcloud, 
                                                      pointcloud_publisher, 
                                                      Welding_Trajectory_publisher);
    //////////////////////////////////////////////////////////////////////////////////////

    
    camera_pointcloud->points.clear();
    cam_pc_transform->points.clear();
    cloud_ptr->points.clear();

    ros::spinOnce(); //allow data update from callback; 
    // naptime.sleep(); // wait for remainder of specified period; 
  }


  // ros::spin();

  return 0;
}


vector< geometry_msgs::Pose > trajectory_6DOF_generation(int &trajectoryPlanning_flag, 
                                                         int &process_count, 
                                                         int process_count_limit, 
                                                         Cloud::Ptr cloud_ptr, 
                                                         Cloud::Ptr cloud_ptr_filter, 
                                                         pcl::PointXYZ realsense_position, 
                                                         ros::Rate naptime, 
                                                         sensor_msgs::PointCloud2 pub_pointcloud, 
                                                         ros::Publisher pointcloud_publisher, 
                                                         ros::Publisher Welding_Trajectory_publisher)
{
  if(trajectoryPlanning_flag == 1)
  {
    process_count++;
    input_pointcloud_filter(process_count, process_count_limit, cloud_ptr, cloud_ptr_filter);

    if(process_count >= process_count_limit)
    {
      trajectoryPlanning_flag = 0; process_count = 0;
      cout << "cloud_ptr_filter->points.size(): " << cloud_ptr_filter->points.size() << endl; 
      cout << "trajectory_planning" << endl;

      vector< geometry_msgs::Pose > Welding_Trajectory ;
      Rviz_TrajectoryPose = trajectory_planning(Welding_Trajectory, cloud_ptr_filter, realsense_position, pub_pointcloud, pointcloud_publisher);

      for(int i = 0; i < Welding_Trajectory.size(); i++)
      {
        Welding_Trajectory_publisher.publish(Welding_Trajectory[i]);
        naptime.sleep();
      }
      cout << "3D path is published !!!!!!!!!" << endl;
    }
  }
 
  return Rviz_TrajectoryPose;
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector< geometry_msgs::Pose > trajectory_planning(vector< geometry_msgs::Pose > &Welding_Trajectory,
                                                  Cloud::Ptr cloud_ptr1, 
                                                  pcl::PointXYZ realsense_position, 
                                                  sensor_msgs::PointCloud2 pub_pointcloud, 
                                                  ros::Publisher pointcloud_publisher)
{
  float seam_detection_radius = 0.01;
  clock_t begin = clock();

  int show_Pointcloud_timeMax = 100;
 
  //Seam Location:    
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "1.读入原始pointcloud" << endl << endl;
  PointCloud::Ptr cloud_ptr_show (new PointCloud);
  Cloud::Ptr cloud_ptr    = read_pointcloud(seam_detection_radius, cloud_ptr_show);
  SurfaceProfile_Reconstruction(seam_detection_radius, cloud_ptr, cloud_ptr_show);
  Cloud::Ptr cloud_ptr_origin = cloud_ptr_origin_copy(cloud_ptr);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "2.算出所有点的法向量" << endl << endl;
  Point3f Cam_Position; Cam_Position.x = realsense_position.x; Cam_Position.y = realsense_position.y; Cam_Position.z = realsense_position.z; 
  vector<Point3f> Normal = PointNormal_Computation(seam_detection_radius, cloud_ptr, cloud_ptr_show, Cam_Position);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "3.分割出所有可能的焊接缝" << endl << endl;
  Delete_SmoothChange_Plane(seam_detection_radius, cloud_ptr, cloud_ptr_show, Normal, pub_pointcloud, pointcloud_publisher);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "4.人工筛选出目标焊接缝" << endl << endl;
  Screen_Candidate_Seam(cloud_ptr, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);  // geometry_msgs::Pose path_point;
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);

  //Trajectory Planning:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "5.提取焊接缝边界" << endl << endl;
  Cloud::Ptr seam_edge = Extract_Seam_edge(cloud_ptr, cloud_ptr_show);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "6.焊接缝三维轨迹点" << endl << endl; 
  Cloud::Ptr PathPoint_Position = PathPoint_Position_Generation(seam_edge, cloud_ptr_origin, cloud_ptr_show);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "7.焊接缝轨迹点的焊枪方向" << endl << endl;
  vector<Point3f> Torch_Normal_Vector = PathPoint_Orientation_Generation(PathPoint_Position, cloud_ptr_origin, cloud_ptr, cloud_ptr_show, Cam_Position);
  show_pointcloud_Rviz(show_Pointcloud_timeMax, cloud_ptr_show, pub_pointcloud, pointcloud_publisher);

  //Trajectory Generation and Publishing:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cout << "8.生成最终6DOF轨迹" << endl << endl;
  vector< geometry_msgs::Pose > Rviz_TrajectoryPose = Ultimate_6DOF_TrajectoryGeneration(Welding_Trajectory, 
                                                                                         PathPoint_Position, 
                                                                                         Torch_Normal_Vector);

  
  cout << endl;
  clock_t end = clock();
  double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
  cout << elapsed_secs << " s" << endl;
    
  return Rviz_TrajectoryPose;
 }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void show_pointcloud_Rviz(int show_Pointcloud_timeMax, 
                          PointCloud::Ptr show_Rviz_cloud, 
                          sensor_msgs::PointCloud2 pub_pointcloud, 
                          ros::Publisher pointcloud_publisher)
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

void publish_pointcloud_Rviz(string coordinate, 
                             PointCloud::Ptr pointloud, 
                             sensor_msgs::PointCloud2 pub_pointcloud, 
                             ros::Publisher pointcloud_publisher)
{
    pcl::toROSMsg(*pointloud, pub_pointcloud);
    pub_pointcloud.header.frame_id = coordinate;// 
    pub_pointcloud.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(pub_pointcloud);
}
