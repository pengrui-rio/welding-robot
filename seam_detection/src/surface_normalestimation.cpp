#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>//点云文件pcd 读写
#include <pcl/visualization/cloud_viewer.h>//点云可视化
#include <pcl/visualization/pcl_visualizer.h>// 高级可视化点云类
#include <pcl/features/normal_3d.h>//法线特征
#include <pcl/kdtree/kdtree_flann.h>//搜索方法
#include <boost/thread/thread.hpp>
 

using namespace std;
// 别名
typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;

typedef pcl::PointXYZ PointType;



void surface_normal_compute()
{
  // 定义　点云对象　指针
  Cloud::Ptr cloud_ptr (new Cloud);

  // 读取点云文件　填充点云对象
  pcl::PCDReader reader;
  reader.read("/home/rick/Documents/a_system/src/seam_detection/save_pcd/sean.pcd", *cloud_ptr);
  if(cloud_ptr==NULL) { cout << "pcd file read err" << endl; return -1;}
  
  cout << "PointCLoud size() " << cloud_ptr->width * cloud_ptr->height
       << " data points " << pcl::getFieldsList (*cloud_ptr) << "." << endl;

// 创建法线估计类====================================
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;//多核 计算法线模型 OpenMP
  ne.setInputCloud (cloud_ptr);

// 添加搜索算法 kdtree search  最近的几个点 估计平面 协方差矩阵PCA分解 求解法线
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // 输出点云 带有法线描述
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>& cloud_normals = *cloud_normals_ptr;
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.01);//半径内搜索临近点
  // ne.setKSearch(8);       //其二 指定临近点数量

  // 计算表面法线特征
  ne.compute (cloud_normals);
  cout << "compute is done!!! " << endl;
  

  cout << "cloud_normals size():" << cloud_normals.size() << endl;

  for(float i = 0; i < cloud_normals.size(); i++)
  {
    // cout << "cloud_normal queriterien: " << cloud_normals[i] << endl;
    // cout << cloud_normals[i].normal_x << " " << cloud_normals[i].normal_y << " " << cloud_normals[i].normal_z << " " << cloud_normals[i].curvature << endl << endl;
 
    cout << "cloud_normals[i].curvature:" << cloud_normals[i].curvature << endl;
  }

  // 点云+法线 可视化
  //pcl::visualization::PCLVisualizer viewer("pcd　viewer");// 显示窗口的名字
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));  
 
  pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_color_handler (cloud_ptr, 200, 0, 0);//红色
  //viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr,cloud_color_handler,"sample cloud");//PointXYZRGB 类型点
  viewer_ptr->addPointCloud<PointType>(cloud_ptr, cloud_color_handler, "original point cloud");//点云标签

  viewer_ptr->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_ptr, cloud_normals_ptr, 5,0.003, "normal");//法线标签
//其中，参数5表示整个点云中每5个点显示一个法向量（若全部显示，可设置为1，  0.02表示法向量的长度，最后一个参数暂时还不知道 如何影响的）);
  viewer_ptr->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original point cloud");
  //渲染属性，可视化工具，3维数据， 其中PCL_VISUALIZER_POINT_SIZE表示设置点的大小为3

  //viewer_ptr->addCoordinateSystem(1.0);//建立空间直角坐标系
  //viewer_ptr->setCameraPosition(0,0,200); //设置坐标原点
  viewer_ptr->initCameraParameters();//初始化相机参数

  while (!viewer_ptr->wasStopped())
  {
      viewer_ptr->spinOnce ();
      pcl_sleep(0.01);
  }

 
}
