#include <ros/ros.h>
#include <main.h>



// 相机内参
double camera_factor = 1000;
double camera_cx = 311.2325744628906;
double camera_cy = 226.9261474609375;
double camera_fx = 619.9661254882812;
double camera_fy = 619.856201171875;


//receive robot current pose flag
bool trajectoryPlanning_flag = false;
int receive_pose_flag = 0, process_count = 0, process_count_limit = 1;
float current_x = 0  , current_y = 0    , current_z = 0;
float current_yaw = 0, current_pitch = 0, current_roll = 0;


 
//callback function:
// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;

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

string dataset_folder_path = "/home/rick/Documents/a_system/src/pointcloud_dataset/box";
int receive_capture_count  = 1;
int process_frame_count    = 1;
void pointcloud_storageFolder_Callback(const std_msgs::String::ConstPtr& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{
  ROS_INFO("I heard the pointcloud_storageFolder"); 
  dataset_folder_path = msg->data.c_str();
  cout << "dataset_folder_path: " << dataset_folder_path << endl;

  receive_capture_count++;
}
 
 
////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  //initial configuration
  ros::init(argc, argv, "trajectory_planning");
  ros::NodeHandle nh;
  ros::Rate naptime(1000); // use to regulate loop rate 

  //subscriber:
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("pointcloud_storageFolder", 1, pointcloud_storageFolder_Callback); //接收到到少次进一次回调
  image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
  image_transport::Subscriber depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_Callback);///camera/depth/image_rect_raw
 
  //publisher:
  ros::Publisher Moveit_path_publisher                = nh.advertise<geometry_msgs::Pose>("Moveit_motion_Path", 1);
  ros::Publisher Welding_Trajectory_publisher         = nh.advertise<geometry_msgs::Pose>("Welding_Trajectory", 1);
  ros::Publisher pointcloud_publisher                 = nh.advertise<sensor_msgs::PointCloud2>("processing_pointcloud", 1);
  ros::Publisher model_pointcloud_display_publisher   = nh.advertise<sensor_msgs::PointCloud2>("model_pointcloud_display", 1);
  ros::Publisher vis_pub                              = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  //pointcloud msgs: 

  sensor_msgs::PointCloud2 pub_model_pointcloud_display;
  sensor_msgs::PointCloud2 pub_path;
  sensor_msgs::PointCloud2 pub_pointcloud;
 
  //pointcloud:
  PointCloud::Ptr camera_pointcloud         (new PointCloud);
  PointCloud::Ptr cam_pc_transform          (new PointCloud);
  PointCloud::Ptr model_pointcloud_display  (new PointCloud);
  Cloud::Ptr      cloud_ptr                 (new Cloud);   
  Cloud::Ptr      cloud_ptr_modelSeam       (new Cloud); 

  //tf:
  tf::TransformListener listener;
  tf::TransformBroadcaster tf_broadcaster;
  tf::StampedTransform transform_baselink2tool0;
  tf::StampedTransform transform_tool02torch;


  //read pointcloud dataset
  int pointcloud_frameNum = count_pointcloud_frameNum(dataset_folder_path);
  build_model_pointcloud(dataset_folder_path, 
                         pointcloud_frameNum,
                         model_pointcloud_display);

  //read trajectory info
  vector< geometry_msgs::Pose > record_rviz_trajecotrypose = 
  read_trajectory_frame("/home/rick/Documents/a_system/src/motion_planning1executing/trajectory_planning/trajectoryRviz_csv/box.csv");
 
  while (ros::ok()) 
  {
    // vector< geometry_msgs::Pose > Rviz_TrajectoryPose = 

    // trajectory_6DOF_generation( read_realtime_pointcloud_frame( dataset_folder_path,
    //                                                             pointcloud_frameNum,
    //                                                             receive_capture_count,
    //                                                             process_frame_count,
    //                                                             trajectoryPlanning_flag,
    //                                                             cloud_ptr),                                 
    //                             cloud_ptr, 
    //                             cloud_ptr_modelSeam, 
                            
    //                             trajectoryPlanning_flag, 
    //                             receive_capture_count,

    //                             pub_pointcloud, 
    //                             pointcloud_publisher, 
    //                             Welding_Trajectory_publisher,
                                
    //                             naptime);

    //////////// 在Rviz里面显示每display_pointsize个点的pose
    for(int i = 0; i < record_rviz_trajecotrypose.size(); i++)
    {
      std::string   markerFrame       = Waypoint_markerName_creation(i);
      tf::Transform waypoint_tranform = Waypoint_markerTransform_creation(i, record_rviz_trajecotrypose[i]);

      tf::StampedTransform waypoint_Marker (waypoint_tranform, ros::Time::now(), "/base_link", markerFrame.c_str());
      tf_broadcaster.sendTransform(waypoint_Marker);
    }

    //////////////////////////////////////////////////////////////////////////////////////
    //publish model_pointcloud
    publish_pointcloud_Rviz("base_link", 
                            model_pointcloud_display, 
                            pub_model_pointcloud_display, 
                            model_pointcloud_display_publisher);

    //////////////////////////////////////////////////////////////////////////////////////


    ros::spinOnce(); //allow data update from callback; 
  }

  return 0;
}

