// #include <iostream>
#include <CAD2pointcloud_transformation.h>
// PCL lib


// void CAD_to_pointcloud(PointCloud::Ptr map_pointcloud)
// {
//   pcl::PolygonMesh mesh;
//   pcl::io::loadPolygonFileOBJ("/home/rick/Documents/a_system/src/CAD2pointcloud/stl_file/straight.obj", mesh);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
//   cout << "cloud->points.size(): "<<  cloud->points.size() << endl;

//   for(float j = 0; j < cloud->points.size(); j++)
//   {
//     pcl::PointXYZRGB p; 
//     p.x = cloud->points[j].x;
//     p.y = cloud->points[j].y;
//     p.z = cloud->points[j].z;
//     p.b = 200;
//     p.g = 200;
//     p.r = 200;

//     map_pointcloud->points.push_back( p );    
//   }
//   cout << "map_pointcloud->points.size(): "<<  map_pointcloud->points.size() << endl;

  // /*+++++++++++++++++++++++++单视角点云获取+++++++++++++++++++++++++++++++*/
	// //读取CAD模型
	// vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	// reader->SetFileName("/home/rick/Documents/a_system/src/CAD2pointcloud/stl_file/straight.STL");
	// reader->Update();
	// vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	// polydata = reader->GetOutput();
	// polydata->GetNumberOfPoints();
 
	// //***单视角点云获取
	// //主要是renderViewTesselatedSphere的参数设定
	// //输入
	// float resx = 256;   //显示视点图窗的X大小  分辨率，值多大，采集的点越多
	// float resy = resx;  //显示视点图窗的Y大小
	// std::vector<pcl::PointCloud<pcl::PointXYZ>, \
  // Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > views_xyz;// 各视点点云对应的XYZ信息
 
	// //输出
	// std::vector<Eigen::Matrix4f,\
  // Eigen::aligned_allocator<Eigen::Matrix4f> > poses;// 从目标坐标变换到视点相机坐标
	// std::vector<float> entropies;//0-1之间，视点看到模型的百分比
 
	// //输入
	// int tesselation_level = 0;//表示在角度下的细分数
	// float view_angle = 90;//虚拟相机的视场
	// float radius_sphere = 1;//radius_sphere半径
	// bool use_vertices = true;//是否采用tessellated icosahedron 的vertices
 
 
  //   //PCLVisualizer 显示
	// pcl::visualization::PCLVisualizer vis;
	// vis.addModelFromPolyData(polydata, "mesh", 0);
	// vis.setRepresentationToSurfaceForAllActors();
	// vis.renderViewTesselatedSphere(resx, resy, views_xyz, poses, entropies, \
  // tesselation_level, view_angle, radius_sphere, use_vertices);//显示个角度点云
 
 	// for (int i = 0; i < views_xyz.size(); i++)
	// {
  //   for(float j = 0; j < views_xyz[i].size(); j++)
  //   {
  //     pcl::PointXYZRGB p; 
  //     p.x = views_xyz[i][j].x;
  //     p.y = views_xyz[i][j].y;
  //     p.z = views_xyz[i][j].z;
  //     p.b = 200;
  //     p.g = 200;
  //     p.r = 200;

  //     map_pointcloud->points.push_back( p );    
  //   }
  //   // cout << "views_xyz[i].size(): "<< i << " " << views_xyz[i].size() << endl;

  //   pcl::PointCloud<pcl::PointXYZ> views_cloud;
  //   pcl::transformPointCloud<pcl::PointXYZ>(views_xyz[i]/*输入点云*/, views_cloud/*输出点云*/, poses[i]/*刚性变换*/);
  //   // cout << "poses[i]: "<< poses[i] << endl; 
  //   for(float j = 0; j < views_cloud.size(); j++)
  //   {
  //     pcl::PointXYZRGB p; 
  //     p.x = views_cloud[j].x;
  //     p.y = views_cloud[j].y;
  //     p.z = views_cloud[j].z;
  //     p.b = 200;
  //     p.g = 200;
  //     p.r = 200;

  //     // cout << "views_cloud[j]: " << views_cloud[j] << endl;
  //     map_pointcloud->points.push_back( p );    
  //   }
  //   // views_cloud->points.clear();
  //   // break;
  // }
  // cout << "map_pointcloud->points.size(): "<<  map_pointcloud->points.size() << endl;

	// //保存
	// for (int i = 0; i < views_xyz.size(); i++)
	// {
	// 	pcl::PointCloud<pcl::PointXYZ> views_cloud;
		// pcl::transformPointCloud<pcl::PointXYZ>(views_xyz[i]/*输入点云*/, views_cloud/*输出点云*/, poses[i]/*刚性变换*/);
 
	// 	std::stringstream ss;
	// 	ss << "cloud_view_" << i << ".ply";
	// 	pcl::io::savePLYFile(ss.str(), views_cloud);
	// }
 
	////// 显示原STL文件
	// while ( ! vis.wasStopped())
	// {
	// 	vis.spinOnce();
	// }
// }



void CAD_csv_to_pointcloud( string trajectoryInfo_folder_path, PointCloud::Ptr map_pointcloud )
{
  
	ifstream inFile(trajectoryInfo_folder_path.c_str());
	string lineStr;
	vector< vector <string> > strArray;
	while (getline(inFile, lineStr))
	{
		// 打印整行字符串
		cout << lineStr << endl;
		// 存成二维表结构
		stringstream ss(lineStr);
		string str;
		vector<string> lineArray;
		// 按照逗号分隔
		while (getline(ss, str, ' '))
			lineArray.push_back(str);
		strArray.push_back(lineArray);
	}
  cout << "strArray.size(): " << strArray.size() << endl << endl;

  for(float i = 0; i < strArray.size(); i++)
  {
    if(strArray[i].size() == 0)
    {
      cout << "empty!!" << endl;
      continue;
    }
    pcl::PointXYZRGB p; 
    p.x = 0;
    p.y = 0;
    p.z = 0;
    p.b = 200;
    p.g = 200;
    p.r = 200;

    stringstream position_x;
    position_x << strArray[i][0];
    position_x >> p.x;
    p.x = p.x / 1000 + 0.1;

    stringstream position_y;
    position_y << strArray[i][1];
    position_y >> p.y;
    p.y = p.y / 1000 + 0.6;

    stringstream position_z;
    position_z << strArray[i][2];
    position_z >> p.z;
    p.z = p.z / 1000 ;

    map_pointcloud->points.push_back( p ); 
    cout << "p: "<<  p << endl;

  }
  cout << "map_pointcloud->points.size(): "<<  map_pointcloud->points.size() << endl;
 
  map_pointcloud->width = 1;
  map_pointcloud->height = map_pointcloud->points.size();

  cout << "map_pointcloud->points.size()" << map_pointcloud->points.size() << endl;
  pcl::PCDWriter writer;
  writer.write("/home/rick/Documents/pcl_sample/ICP/ICP_cylinder.pcd", *map_pointcloud, false) ;

}


void read_pointcloud_for_icp(string PCD_folder_path, Cloud::Ptr cloud_ptr_in)
{
  pcl::PCDReader reader;
  reader.read(PCD_folder_path, *cloud_ptr_in);
}

void transform_workpiece_pointcloud_model(Cloud::Ptr workpiece_pointcloud_model)
{
  // float R = 150, P = 0, Y = 0;
  float R = 0, P = 180, Y = -20;
  Eigen::Vector3d ea(R * M_PI / 180.0, P * M_PI / 180.0, Y * M_PI / 180.0);

  //3.1 欧拉角转换为旋转矩阵
  Eigen::Matrix3d rotation_matrix3;
  rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                     Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                     Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
  cout << "rotation matrix3 =\n" << rotation_matrix3 << endl;   


  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  transformation_matrix (0, 0) = rotation_matrix3(0, 0);
  transformation_matrix (0, 1) = rotation_matrix3(0, 1);
  transformation_matrix (0, 2) = rotation_matrix3(0, 2);
  transformation_matrix (1, 0) = rotation_matrix3(1, 0);
  transformation_matrix (1, 1) = rotation_matrix3(1, 1);
  transformation_matrix (1, 2) = rotation_matrix3(1, 2);
  transformation_matrix (2, 0) = rotation_matrix3(2, 0);
  transformation_matrix (2, 1) = rotation_matrix3(2, 1);
  transformation_matrix (2, 2) = rotation_matrix3(2, 2);

  // Z轴的平移向量 (0.4 meters)
  transformation_matrix (0, 3) = 0;
  transformation_matrix (1, 3) = 0;
  transformation_matrix (2, 3) = 0.3;

  Cloud::Ptr   workpiece_pointcloud_model_temp  (new Cloud);  
  cout << "Applying this rigid transformation to: workpiece_pointcloud_model -> pointcloud_model_target" << endl;
  pcl::transformPointCloud (*workpiece_pointcloud_model, *workpiece_pointcloud_model_temp, transformation_matrix);

  workpiece_pointcloud_model->points.clear();

  *workpiece_pointcloud_model = *workpiece_pointcloud_model_temp;

 }


void create_target_pointcloudModel(Cloud::Ptr pointcloud_model_target, Cloud::Ptr workpiece_pointcloud_model)
{
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  double theta = M_PI / 42;  // 旋转的角度用弧度的表示方法
  transformation_matrix (0, 0) = cos (theta);
  transformation_matrix (0, 1) = -sin (theta); 
  transformation_matrix (1, 0) = sin (theta);
  transformation_matrix (1, 1) = cos (theta);
  // Z轴的平移向量 (0.4 meters)
  transformation_matrix (0, 3) = 0.4;
  transformation_matrix (1, 3) = 0.3;
  transformation_matrix (2, 3) = -0.1;

  cout << "Applying this rigid transformation to: workpiece_pointcloud_model -> pointcloud_model_target" << endl;
  pcl::transformPointCloud (*workpiece_pointcloud_model, *pointcloud_model_target, transformation_matrix);
}

pcl::IterativeClosestPoint<PointType, PointType> ICP_config(Cloud::Ptr pointcloud_model_target,
                                                            Cloud::Ptr workpiece_pointcloud_model)
{
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaximumIterations (1);
  icp.setInputSource (workpiece_pointcloud_model);
  icp.setInputTarget (pointcloud_model_target);

  return icp;
}
 
void ICP_registration(int iterations, 
                      pcl::IterativeClosestPoint<PointType, PointType> icp,
                      Cloud::Ptr pointcloud_model_target, 
                      Cloud::Ptr workpiece_pointcloud_model)
{
  pcl::console::TicToc time;

  time.tic ();
  icp.align (*workpiece_pointcloud_model);
  cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << endl;

  if (icp.hasConverged ())
  {
    cout << "ICP has converged, score is " << icp.getFitnessScore () << endl << endl;
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
  }
 
}



// void ICP_registration(string PCD_folder_path, PointCloud::Ptr map_pointcloud)
// {
    // The point clouds we will be using
  // Cloud::Ptr cloud_in (new Cloud);  // Original point cloud
  // Cloud::Ptr cloud_tr (new Cloud);  // Transformed point cloud
  // Cloud::Ptr cloud_icp (new Cloud);  // ICP output point cloud


  // int iterations = 1;  // Default number of ICP iterations

  // pcl::console::TicToc time;
  // time.tic ();
 
  // pcl::PCDReader reader;
  // reader.read(PCD_folder_path, *cloud_in);

  // Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  // double theta = M_PI / 8;  // 旋转的角度用弧度的表示方法
  // transformation_matrix (0, 0) = cos (theta);
  // transformation_matrix (0, 1) = -sin (theta); 
  // transformation_matrix (1, 0) = sin (theta);
  // transformation_matrix (1, 1) = cos (theta);

  // // Z轴的平移向量 (0.4 meters)
  // transformation_matrix (2, 3) = 0.4;


  // // Display in terminal the transformation matrix
  // std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  // print4x4Matrix (transformation_matrix);

  // // Executing the transformation
  // pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
  // *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

  // // The Iterative Closest Point algorithm
  // time.tic ();
  // pcl::IterativeClosestPoint<PointType, PointType> icp;
  // icp.setMaximumIterations (iterations);
  // icp.setInputSource (cloud_icp);
  // icp.setInputTarget (cloud_in);
  // icp.align (*cloud_icp);
  // std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

  // if (icp.hasConverged ())
  // {
  //   std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
  //   std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
  //   transformation_matrix = icp.getFinalTransformation ().cast<double>();
  //   print4x4Matrix (transformation_matrix);
  // }
  // else
  // {
  //   PCL_ERROR ("\nICP has not converged.\n");
  //   // return (-1);
  // }

  // // // Visualization
  // pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // // // Create two vertically separated viewports
  // // int v1 (0);
  // int v2 (1);
  // // viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  // viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // // The color we will be using
  // float bckgr_gray_level = 0.0;  // Black
  // float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // // Original point cloud is white
  // pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
  //                                                                            (int) 255 * txt_gray_lvl);
  // // viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
  // viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

  // // // // Transformed point cloud is green
  // // // pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_tr_color_h (cloud_tr, 20, 180, 20);
  // // // viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

  // // ICP aligned point cloud is red
  // pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_icp_color_h (cloud_icp, 180, 20, 20);
  // viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

  // // // Adding text descriptions in each viewport
  // // viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  // viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  // std::stringstream ss;
  // ss << iterations;
  // std::string iterations_cnt = "ICP iterations = " + ss.str ();
  // viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // // // Set background color
  // // viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  // viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // // Set camera position and orientation
  // viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  // viewer.setSize (1280, 1024);  // Visualiser window size

  // // // Register keyboard callback :
  // // viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
  // // cout <<  "press the space" << std::endl;
 

  // // Display the visualiser
  // // 显示
  // while (ros::ok())
  // {
  //   viewer.spinOnce ();

  //   // //按下空格键的函数
  //   // if (next_iteration)
  //   // {
  //   // 最近点迭代算法
  //   time.tic ();
  //   icp.align (*cloud_icp);
  //   std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

  //   if (icp.hasConverged ())
  //   {
  //     // printf ("\033[11A");  // Go up 11 lines in terminal output.
  //     // printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
  //     cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << endl;
  //     // transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate!
  //     // print4x4Matrix (transformation_matrix);  // 打印矩阵变换

  //     // ss.str ("");
  //     // ss << iterations;
  //     // std::string iterations_cnt = "ICP iterations = " + ss.str ();
  //     // viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
  //     viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
  //   }
  //   else
  //   {
  //     PCL_ERROR ("\nICP has not converged.\n");
  //   }
  //   // }
  //   // next_iteration = false;
  // }


// }