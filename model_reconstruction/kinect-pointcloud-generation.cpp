#include <ros/ros.h>
#include <pcl/point_cloud.h>
//msgs type and conversion
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//pcd io
#include <pcl/io/pcd_io.h>
//point types
#include <pcl/point_types.h>
 
int i=0;
char ch=(char)(int('0'));
void call_back(const sensor_msgs::PointCloud2ConstPtr& input)
{
//Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);//cloud is the output
    
//save to PCD for 5 times    
    std::string pcd_name("pcl_2_pcd");
    if(i<5)
    {
        i++;
        ch++;
        pcd_name+=ch;
        if(pcl::io::savePCDFileASCII ("/home/kyle/Documents/mesh_sample/eye_in_hand2/realsense.pcd", cloud)>=0)
        //input pointcloud should be pcl::PointCloud<PointT> only,rather than others 
        {
            std::cerr << "Saved  " << pcd_name<<".pcd"<< std::endl;
        }
    
    }
}
 
int
main(int argc,char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "pcl_2_pcd");
  ros::NodeHandle nh;
 
// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, call_back);
 
// Spin
  ros::spin ();
}
 