#include <algorithm.h>


Cloud::Ptr read_pointcloud (void)
{
  //seam detection
  Cloud::Ptr cloud_ptr (new Cloud);

  pcl::PCDReader reader;
  reader.read("/home/rick/Documents/a_system/src/seam_detection/save_pcd/test.pcd", *cloud_ptr);
  
  cout << "PointCLoud size() " << cloud_ptr->width * cloud_ptr->height
       << " data points " << pcl::getFieldsList (*cloud_ptr) << "." << endl << endl;

  return cloud_ptr;
}

