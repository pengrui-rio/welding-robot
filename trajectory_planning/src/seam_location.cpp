#include <seam_location.h>

 
void swap(int array[], int i, int j)
{
	int temp = array[i];
	array[i] = array[j];
	array[j] = temp;
}
void BubbleSort1(int array[], int n)
{
	for (int i = 0; i < n-1; i++)
	{
		for (int j = i + 1; j < n-1; j++)
		{
			if (array[i]>array[j])
				swap(array, j, i);//每次i后面的元素比array[i]小就交换。
		}
	}
}



// void Obtain_pathPoint_BaseCoor(Point3f path_point_3D)
// {
//   // Base -> Camera
// 	Matrix4d T_B_C; 

//   T_B_C << 1,            0,           -0,   -0.0405463,
//            0,           -1, -8.74228e-08,     0.443597,
//            0,  8.74228e-08,           -1,     0.830352,
//            0,            0,            0,            1;

//   // Camera -> object
// 	Matrix4d T_C_o; 
//   float T_C_o_Q[4];
//   euler_to_quaternion(0, 180, 0, T_C_o_Q);
//   float T_C_o_r[9];
//   Quaternion_to_RotationMatrix(T_C_o_Q[0], T_C_o_Q[1], T_C_o_Q[2], T_C_o_Q[3], T_C_o_r);//camera_color_optical_frame
//   // euler_to_RotationMatrix(-90, -180, 0, T_C_o_r);
// 	T_C_o << T_C_o_r[0], T_C_o_r[1], T_C_o_r[2], path_point_3D.x,
//            T_C_o_r[3], T_C_o_r[4], T_C_o_r[5], path_point_3D.y,
//            T_C_o_r[6], T_C_o_r[7], T_C_o_r[8], path_point_3D.z,
//            0,          0,          0,          1;
//   // cout << "T_C_o: " << endl << T_C_o << endl;


//   // Base -> object
// 	Matrix4d T_B_o; 

//   T_B_o = T_B_C * T_C_o;
//   cout << "T_B_o: " << endl << T_B_o << endl;
//   // cout << endl;

// }


// void RGBimage_seam_extration(Mat color_pic, Mat depth_pic)
// {
//   float xMin = 180, xMax = 530;
//   float yMin = 140, yMax = 300;

//   color_pic(cv::Rect(xMin,yMin,xMax-xMin,yMax-yMin)).copyTo(color_pic);
 
//   // 遍历彩色图
//   // InterestImage.copyTo(result_image) ;//拷贝

//   for (int m = 0; m < color_pic.rows; m++)  //160
//   {
//     for (int n = 0; n < color_pic.cols; n++) //350
//     {
//       // if((n == 22  && m == 67)  ||
//       //    (n == 50  && m == 88)  ||
//       //    (n == 83  && m == 105) ||
//       //    (n == 124 && m == 115) ||
//       //    (n == 172 && m == 104) ||
//       //    (n == 223 && m == 80)  ||
//       //    (n == 273 && m == 67)  ||
//       //    (n == 301 && m == 78)  ||
//       //    (n == 330 && m == 115) )
//       // {
//       //   color_pic.ptr<uchar>(m)[n*3]   = 0;
//       //   color_pic.ptr<uchar>(m)[n*3+1] = 0;
//       //   color_pic.ptr<uchar>(m)[n*3+2] = 200;
//       // }

//       // if( m == 80 || m == 72 )
//       // {
//       //   color_pic.ptr<uchar>(m)[n*3]   = 0;
//       //   color_pic.ptr<uchar>(m)[n*3+1] = 0;
//       //   color_pic.ptr<uchar>(m)[n*3+2] = 200;
//       // }

//     }
//   }
//   cv::imshow("crop_image", color_pic);


//   vector<Point2i> path_point;
//   Point2i p;
//   p.x = 22  + xMin; p.y = 67  + yMin; path_point.push_back(p);
//   p.x = 50  + xMin; p.y = 88  + yMin; path_point.push_back(p);
//   p.x = 83  + xMin; p.y = 105 + yMin; path_point.push_back(p);
//   p.x = 124 + xMin; p.y = 115 + yMin; path_point.push_back(p);
//   p.x = 172 + xMin; p.y = 104 + yMin; path_point.push_back(p);
//   p.x = 223 + xMin; p.y = 80  + yMin; path_point.push_back(p);
//   p.x = 273 + xMin; p.y = 67  + yMin; path_point.push_back(p);
//   p.x = 301 + xMin; p.y = 78  + yMin; path_point.push_back(p);
//   p.x = 330 + xMin; p.y = 115 + yMin; path_point.push_back(p);

//   for (int i = 0; i < path_point.size(); i++)  //160
//   {
//     Point3f p;
//     float d = depth_pic.ptr<float>(path_point[i].y)[path_point[i].x]; 
//     p.z = double(d) / camera_factor;
//     p.x = (path_point[i].x - camera_cx) * p.z / camera_fx; 
//     p.y = (path_point[i].y - camera_cy) * p.z / camera_fy;

//     cout << "i-th: " << i+1 << endl;
//     Obtain_pathPoint_BaseCoor(p);
//   }

  

	// Mat image;
	// GaussianBlur(InterestImage,image,Size(3,3),0);
	// Canny(InterestImage,image,70,180);
	// vector<vector<Point> > contours;
	// vector<Vec4i> hierarchy;
	// findContours(image,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());
	// Mat imageContours=Mat::zeros(image.size(),CV_8UC1);
	// Mat Contours=Mat::zeros(image.size(),CV_8UC1);  //绘制
	// for(int i=0;i<contours.size();i++)
	// {
	// 	//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
	// 	for(int j=0;j<contours[i].size();j++) 
	// 	{
	// 		//绘制出contours向量内所有的像素点
	// 		Point P=Point(contours[i][j].x,contours[i][j].y);
	// 		Contours.at<uchar>(P)=255;
	// 	}
 
	// 	//输出hierarchy向量内容
	// 	char ch[256];
	// 	sprintf(ch,"%d",i);
	// 	string str=ch;
	// 	cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;
 
	// 	//绘制轮廓
	// 	drawContours(imageContours,contours,i,Scalar(255),1,8,hierarchy);
	// }
	// imshow("Contours Image",imageContours); //轮廓
	// imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集
 
// }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


Cloud::Ptr read_pointcloud (PointCloud::Ptr cloud_ptr_show)
{
  //seam detection
  Cloud::Ptr cloud_ptr (new Cloud);

  // PCD reader
  pcl::PCDReader reader;
  reader.read("/home/rick/Documents/a_system/src/trajectory_planning/src/input_cloud.pcd", *cloud_ptr);
  
  cout << "PointCLoud size() " << cloud_ptr->width * cloud_ptr->height
       << " data points " << pcl::getFieldsList (*cloud_ptr) << "." << endl;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal> mls_points;
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  mls.setInputCloud (cloud_ptr);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.01);
  mls.process (mls_points);
  cout << "smooth size(): " <<  mls_points.size() << endl << endl;

  Cloud::Ptr smooth_cloud ( new Cloud );
  for(float i = 0; i < mls_points.size(); i++)
  {
    pcl::PointXYZ p;
    p.x = mls_points[i].x; 
    p.y = mls_points[i].y;
    p.z = mls_points[i].z;

    smooth_cloud->points.push_back( p );
  }

  for(float i = 0; i < smooth_cloud->points.size(); i++)
  {
    pcl::PointXYZRGB p;
    p.x = smooth_cloud->points[i].x;
    p.y = smooth_cloud->points[i].y;
    p.z = smooth_cloud->points[i].z;
    p.b = 200; 
    p.g = 200;
    p.r = 200;
    cloud_ptr_show->points.push_back( p );    
  }


  // //bottom_straight
  // for(float i = 0; i < cloud_ptr->points.size(); i++)
  // {
  //   pcl::PointXYZRGB p;

  //   p.x = cloud_ptr->points[i].x ; 
  //   p.y = cloud_ptr->points[i].y ;
  //   p.z = cloud_ptr->points[i].z ;//- 0.457; // 0.125
  //   p.b = 200; 
  //   p.g = 200;
  //   p.r = 200;
  //   cloud_ptr_show->points.push_back( p );    
  // }

  //这个不用动
  cloud_ptr->clear();
  for(float i = 0; i < cloud_ptr_show->points.size(); i++)
  {
    pcl::PointXYZ p;
    p.x = cloud_ptr_show->points[i].x; 
    p.y = cloud_ptr_show->points[i].y;
    p.z = cloud_ptr_show->points[i].z;
    cloud_ptr->points.push_back( p );    
  }

  cout << "cloud_ptr_show->points.size()" << cloud_ptr->points.size() << endl << endl;

  return cloud_ptr;
}


void SurfaceProfile_Reconstruction(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal> mls_points;
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  mls.setInputCloud (cloud_ptr);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.01);
  mls.process (mls_points);
  cout << "smooth size(): " <<  mls_points.size() << endl << endl;

  Cloud::Ptr smooth_cloud ( new Cloud );
  for(float i = 0; i < mls_points.size(); i++)
  {
    pcl::PointXYZ p;
    p.x = mls_points[i].x; 
    p.y = mls_points[i].y;
    p.z = mls_points[i].z;

    smooth_cloud->points.push_back( p );
  }

  cloud_ptr_show->clear();
  for(float i = 0; i < smooth_cloud->points.size(); i++)
  {
    pcl::PointXYZRGB p;
    p.x = smooth_cloud->points[i].x;
    p.y = smooth_cloud->points[i].y;
    p.z = smooth_cloud->points[i].z;
    p.b = 200; 
    p.g = 200;
    p.r = 200;
    cloud_ptr_show->points.push_back( p );    
  }


  //这个不用动
  cloud_ptr->clear();
  for(float i = 0; i < cloud_ptr_show->points.size(); i++)
  {
    pcl::PointXYZ p;
    p.x = cloud_ptr_show->points[i].x; 
    p.y = cloud_ptr_show->points[i].y;
    p.z = cloud_ptr_show->points[i].z;
    cloud_ptr->points.push_back( p );    
  }

  cout << "cloud_ptr_show->points.size()" << cloud_ptr->points.size() << endl << endl;

}


vector<Point3f> Pointnormal_Direction_Unify(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector<Point3f> Normal, Point3f Cam_Position);
vector<Point3f> PointNormal_Computation(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, Point3f Cam_Position)
{
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZRGB p;

    p.x = cloud_ptr->points[i].x ; 
    p.y = cloud_ptr->points[i].y ;
    p.z = cloud_ptr->points[i].z ;//- 0.457; // 0.125
    p.b = 200; 
    p.g = 200;
    p.r = 200;
    cloud_ptr_show->points.push_back( p );    
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // define kdtree
  float radius = 0.005;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  // 创建一个 KdTree 对象
  kdtree.setInputCloud (cloud_ptr);  // 将前面创建的随机点云作为 KdTree 输入
  vector<int> pointIdxRadiusSearch; // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  vector<float> pointRadiusSquaredDistance;

  // kdtree.radiusSearch (cloud_ptr->points[132151], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);  //132151
  // for(float j = 0; j < pointIdxRadiusSearch.size(); j++)
  // {
  //   cloud_ptr_show->points[pointIdxRadiusSearch[j]].b = 200; 
  //   cloud_ptr_show->points[pointIdxRadiusSearch[j]].g = 0;
  //   cloud_ptr_show->points[pointIdxRadiusSearch[j]].r = 0;
  // }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //计算所有点的法向量
  vector<Point3f> Normal;
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  {
    kdtree.radiusSearch (cloud_ptr->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);  //132151
    
    //求质心：
    Point3f Pm;
    for(float j = 0; j < pointIdxRadiusSearch.size(); j++)
    {
      float xj = cloud_ptr->points[pointIdxRadiusSearch[j]].x;
      float yj = cloud_ptr->points[pointIdxRadiusSearch[j]].y;
      float zj = cloud_ptr->points[pointIdxRadiusSearch[j]].z;

      Pm.x += xj;
      Pm.y += yj;
      Pm.z += zj;
    }
    Pm.x = Pm.x / pointIdxRadiusSearch.size();
    Pm.y = Pm.y / pointIdxRadiusSearch.size();
    Pm.z = Pm.z / pointIdxRadiusSearch.size();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //求每个向量：
    vector<Point3f> V;
    for( float j = 0; j < pointIdxRadiusSearch.size(); j++)
    { 
      float xj = cloud_ptr->points[pointIdxRadiusSearch[j]].x;
      float yj = cloud_ptr->points[pointIdxRadiusSearch[j]].y;
      float zj = cloud_ptr->points[pointIdxRadiusSearch[j]].z;

      Point3f v;
      v.x = Pm.x - xj;
      v.y = Pm.y - yj;
      v.z = Pm.z - zj;
      V.push_back(v);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //构建协方差矩阵
    Matrix3f yyT;
    yyT << 0,0,0,
          0,0,0,
          0,0,0;

    for( float j = 0; j < V.size(); j++)
    {
      yyT(0,0) += V[j].x * V[j].x, yyT(0,1) += V[j].x * V[j].y, yyT(0,2) += V[j].x * V[j].z;
      yyT(1,0) += V[j].y * V[j].x, yyT(1,1) += V[j].y * V[j].y, yyT(1,2) += V[j].y * V[j].z;
      yyT(2,0) += V[j].z * V[j].x, yyT(2,1) += V[j].z * V[j].y, yyT(2,2) += V[j].z * V[j].z;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //SVD分解，特征值最小的一列向量位于U的最后一列
    JacobiSVD<Eigen::MatrixXf> svd(yyT, ComputeThinU | ComputeThinV );  
    Matrix3f  U = svd.matrixU();   
    // Matrix3f V1 = svd.matrixV();                    
    // Matrix3f S = U.inverse() * yyT * V1.transpose().inverse(); // S = U ^ -1 * A * VT * -1  
    // cout<<"yyT :\n" << yyT  << endl<< endl;  
    // cout<<"U :\n"   << U << endl << endl;                               
    // Point3f ;
    // cout<<"S :\n"   << S   << endl<< endl;  
    // cout<<"V1 :\n"  << V1  << endl<< endl;  
    // cout<<"U * S * VT :\n" << U * S * V1.transpose() << endl;  

    Point3f normal;
    normal.x = U(0,2);
    normal.y = U(1,2);
    normal.z = U(2,2);
    Normal.push_back(normal);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  // cout << "Normal:\n" << Normal << endl;
  // cout << "size:" << Normal.size() << endl; 

  // Point3f Cam_Position; Cam_Position.x = 0, Cam_Position.y = 0, Cam_Position.z = 0;

  return  Pointnormal_Direction_Unify(cloud_ptr, cloud_ptr_show, Normal, Cam_Position);
}


vector<Point3f> Pointnormal_Direction_Unify(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector<Point3f> Normal, Point3f Cam_Position)
{
  //求标准向量：点到相机坐标系
  vector<Point3f> Vector_point_to_camPosi;
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  {
    Point3f standard_vector;
    standard_vector.x = Cam_Position.x - cloud_ptr->points[i].x;
    standard_vector.y = Cam_Position.y - cloud_ptr->points[i].y;
    standard_vector.z = Cam_Position.z - cloud_ptr->points[i].z;
    Vector_point_to_camPosi.push_back(standard_vector);
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //求法向量与标准向量的夹角
  vector<float> Theta;
  for(float i = 0; i < Normal.size(); i++)
  {
    float a_b = Normal[ i ].x * Vector_point_to_camPosi[ i ].x +
                Normal[ i ].y * Vector_point_to_camPosi[ i ].y +
                Normal[ i ].z * Vector_point_to_camPosi[ i ].z ;  

    float a2 = sqrt(pow(Normal[ i ].x, 2) +
                    pow(Normal[ i ].y, 2) +
                    pow(Normal[ i ].z, 2));  

    float b2 = sqrt(pow(Vector_point_to_camPosi[ i ].x, 2) +
                    pow(Vector_point_to_camPosi[ i ].y, 2) +
                    pow(Vector_point_to_camPosi[ i ].z, 2)); 

    float COS_ab = a_b / (a2 * b2) ;

    // float theta = acos( COS_ab ) * 180.0 / M_PI ;

    // cout << "theta: " << theta << endl;
    Theta.push_back(COS_ab);
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //大于90度的需要反过来
  for(float i = 0; i < Normal.size(); i++)
  {
    if(Theta[i] <= 0)
    {
      Normal[i] = Normal[i] * (-1);
    }
  }

  // cout << "Normal:\n" << Normal << endl;
  cout << "size:" << Normal.size() << endl << endl; 
  
  return  Normal; 
}


void Delete_SmoothChange_Plane(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector<Point3f> Normal, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher)
{
  // for(float i = 0; i < cloud_ptr->points.size(); i++)
  // {
  //   pcl::PointXYZRGB p;

  //   p.x = cloud_ptr->points[i].x ; 
  //   p.y = cloud_ptr->points[i].y ;
  //   p.z = cloud_ptr->points[i].z ;//- 0.457; // 0.125
  //   p.b = 200; 
  //   p.g = 200;
  //   p.r = 200;
  //   cloud_ptr_show->points.push_back( p );    
  // }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // define kdtree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  // 创建一个 KdTree 对象
  kdtree.setInputCloud (cloud_ptr);  // 将前面创建的随机点云作为 KdTree 输入
  vector<int> pointIdxRadiusSearch; // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  vector<float> pointRadiusSquaredDistance;
  float radius = 0.006;
 
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //求每个点的领域方向方差
  vector<float> PointVariance;
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  {
    kdtree.radiusSearch (cloud_ptr->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);  //132151

    float theta = 0, ave_theta = 0;
    vector<float> PointTheta;
    for(float j = 0; j < pointIdxRadiusSearch.size(); j++)
    {
      float a_b = Normal[ pointIdxRadiusSearch[0] ].x * Normal[ pointIdxRadiusSearch[j] ].x +
                  Normal[ pointIdxRadiusSearch[0] ].y * Normal[ pointIdxRadiusSearch[j] ].y +
                  Normal[ pointIdxRadiusSearch[0] ].z * Normal[ pointIdxRadiusSearch[j] ].z ;  

      ave_theta += a_b;
      PointTheta.push_back(a_b);
    }
    ave_theta = ave_theta / pointIdxRadiusSearch.size();

    float variance = 0;
    for(float j = 0; j < pointIdxRadiusSearch.size(); j++)
    {
      variance += pow((PointTheta[j] - ave_theta), 2);
    }
    variance = variance / pointIdxRadiusSearch.size() ;

    PointVariance.push_back(variance);
  }
  // for(float j = 0; j < PointVariance.size(); j++)
  // {
  //   cout << "PointVariance: " << PointVariance[j] << endl;
  // }
  cout << "size:" << PointVariance.size() << endl << endl; 

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //求每个点的平面变化描述子
  vector<float> PointDescriptor;
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  {
    kdtree.radiusSearch (cloud_ptr->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);  //132151

    float alpha = 0, ave_alpha = 0;
    vector<float>  descriptor;
    for(float j = 0; j < pointIdxRadiusSearch.size(); j++)
    {  
      ave_alpha += PointVariance[pointIdxRadiusSearch[j]];
      alpha = PointVariance[pointIdxRadiusSearch[j]];
      descriptor.push_back(alpha);
    }
    ave_alpha = ave_alpha / pointIdxRadiusSearch.size();

    float variance = 0;
    for(float j = 0; j < pointIdxRadiusSearch.size(); j++)
    {
      variance += pow((descriptor[j] - ave_alpha), 2);
    }
    variance = variance / pointIdxRadiusSearch.size() ;

    PointDescriptor.push_back(variance);
  }
  // for(float j = 0; j < PointDescriptor.size(); j++)
  // {
  //   cout << "PointDescriptor: " << PointDescriptor[j] << endl;
  // }
  cout << "size:" << PointDescriptor.size() << endl << endl; 


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //求描述子的最大值最小值, 将描述子约束在0-1之间
  float PointDescriptor_max = 0, PointDescriptor_min = 0;
  for(float j = 0; j < PointDescriptor.size(); j++)
  { 
    if(j == 0)
    {
      PointDescriptor_max = PointDescriptor[0];
      PointDescriptor_min = PointDescriptor[0];
    }

    if (PointDescriptor_max < PointDescriptor[j])
    {
      PointDescriptor_max = PointDescriptor[j];
    }

    if(PointDescriptor_min > PointDescriptor[j])
    {
      PointDescriptor_min = PointDescriptor[j];
    }
  }
  cout << "PointDescriptor_max: "    << PointDescriptor_max << endl;
  cout << "PointDescriptor_min: "    << PointDescriptor_min << endl;

  // for(float j = 0; j < PointDescriptor.size(); j++)
  // { 
  //   PointDescriptor[j] = PointDescriptor[j] / (PointDescriptor_max - PointDescriptor_min);
  // }

  // //打印PointDescriptor
  // for(float j = 0; j < PointDescriptor.size(); j++)
  // {
  //   cout << "PointDescriptor: " << PointDescriptor[j] << endl;
  // }
  // cout << "size:" << PointDescriptor.size() << endl; 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //将变化率高的部分分离出来
  while(ros::ok())
  {
    cout << "PLease input screen_threshold: ";
    float screen_threshold = 0;
    cin >> screen_threshold;

    for(float i = 0; i < cloud_ptr->points.size(); i++)
    {
      if(PointDescriptor[i] >= screen_threshold)
      {
        pcl::PointXYZRGB p;

        p.x = cloud_ptr->points[i].x ; 
        p.y = cloud_ptr->points[i].y ;
        p.z = cloud_ptr->points[i].z ;//- 0.457; // 0.125
        p.b = 200; 
        p.g = 200;
        p.r = 200;
        cloud_ptr_show->points.push_back( p );    
      }
    }
    cout << "cloud_ptr->points.size(): "  << cloud_ptr->points.size() << endl ;
    cout << "cloud_ptr_show->points.size(): "  << cloud_ptr_show->points.size() << endl ;

    pcl::toROSMsg(*cloud_ptr_show, pub_pointcloud);
    pub_pointcloud.header.frame_id = "base_link";
    pub_pointcloud.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(pub_pointcloud);

    cout << endl << "Keep the pointcloud or not? yes or xxxx ";
    string keep_flag;
    cin >> keep_flag;
    if(keep_flag == "yes")
    {
      //后续的输入点云
      cloud_ptr->clear();
      for(float i = 0; i < cloud_ptr_show->points.size(); i++)
      {
        pcl::PointXYZ p;
        p.x = cloud_ptr_show->points[i].x; 
        p.y = cloud_ptr_show->points[i].y;
        p.z = cloud_ptr_show->points[i].z;
        cloud_ptr->points.push_back( p );    
      }
      break;
    }
    
    cloud_ptr_show->clear();
    cout << endl;

  }

}


void Screen_Candidate_Seam(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, sensor_msgs::PointCloud2 pub_pointcloud, ros::Publisher pointcloud_publisher)
{
  cout << "ec_tree_cloud->points.size(): "  << cloud_ptr->points.size() << endl ;
  // push cloud_tree_rm_irrelativePoint into ec_tree_cloud
//   Cloud::Ptr ec_tree_cloud (new Cloud);
//   for(float i = 0; i < cloud_ptr->points.size(); i++)
//   { 
//     pcl::PointXYZ p;

//     p.x = cloud_ptr->points[ i ].x;
//     p.y = cloud_ptr->points[ i ].y;
//     p.z = cloud_ptr->points[ i ].z;

//     ec_tree_cloud->points.push_back( p );
//   }
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //define kdtree for euclidean clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr ec_tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ec_tree->setInputCloud (cloud_ptr);//创建点云索引向量，用于存储实际的点云信息

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> EC;

  EC.setClusterTolerance (0.002); //设置近邻搜索的搜索半径为2cm
  EC.setMinClusterSize (500);//设置一个聚类需要的最少点数目为100
  EC.setMaxClusterSize (10000000); //设置一个聚类需要的最大点数目为25000
  EC.setSearchMethod (ec_tree);//设置点云的搜索机制
  EC.setInputCloud (cloud_ptr);// input cloud
  EC.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中
  
  cout << "ec_tree_cloud->points.size(): "  << cloud_ptr->points.size() << endl ;
  cout << "cluster_indices.size(): " << cluster_indices.size() << endl;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // put largest cluster into final seam cloud
  vector < vector <float> > seam_cluster_all;
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    Cloud::Ptr cloud_cluster (new Cloud);
    
    // push back
    vector<float> seam_cluster;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      seam_cluster.push_back(*pit);
      cloud_cluster->points.push_back (cloud_ptr->points[*pit]);  
    }

    seam_cluster_all.push_back(seam_cluster);
  }
  cout << "seam_cluster_all.size(): " << seam_cluster_all.size() << endl;
  // cout << "seam_cluster_all[0].size(): " << seam_cluster_all[0].size() << endl;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // for(float i = 0; i < seam_cluster_all[0].size(); i++)
  // { 
  //   pcl::PointXYZRGB p;
  //   p.x = cloud_ptr->points[ seam_cluster_all[0][i] ].x;
  //   p.y = cloud_ptr->points[ seam_cluster_all[0][i] ].y;
  //   p.z = cloud_ptr->points[ seam_cluster_all[0][i] ].z;
  //   p.b = 200;
  //   p.g = 200;
  //   p.r = 200;

  //   cloud_ptr_show->points.push_back( p ); 
  // }

  int seam_label = 0;
  while(ros::ok())
  {
    cout << "PLease input index of seam cluster: 0-max";
    cout << "seam_cluster_all.size(): " << seam_cluster_all.size() << endl;

    float index_seam_cluster = 0;
    cin >> index_seam_cluster;

    for(float i = 0; i < seam_cluster_all[index_seam_cluster].size(); i++)
    { 
      pcl::PointXYZRGB p;
      p.x = cloud_ptr->points[ seam_cluster_all[index_seam_cluster][i] ].x;
      p.y = cloud_ptr->points[ seam_cluster_all[index_seam_cluster][i] ].y;
      p.z = cloud_ptr->points[ seam_cluster_all[index_seam_cluster][i] ].z;
      p.b = 200;
      p.g = 200;
      p.r = 200;

      cloud_ptr_show->points.push_back( p ); 
    }

    pcl::toROSMsg(*cloud_ptr_show, pub_pointcloud);
    pub_pointcloud.header.frame_id = "base_link";
    pub_pointcloud.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(pub_pointcloud);

    cout << endl << "Keep the pointcloud or not? yes or xxxx ";
    string keep_flag;
    cin >> keep_flag;
    if(keep_flag == "yes")
    {
      seam_label = index_seam_cluster;
      break;
    }
    
    cloud_ptr_show->clear();
    cout << endl;
  }

  cloud_ptr->clear();
  for(float i = 0; i < seam_cluster_all[seam_label].size(); i++)
  { 
    pcl::PointXYZ p;
    p.x = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].x;
    p.y = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].y;
    p.z = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].z;

    cloud_ptr->points.push_back( p ); 
  }

  // cloud_ptr->width = 1;
  // cloud_ptr->height = cloud_ptr->points.size();

  // cout << "cloud_ptr->points.size()" << cloud_ptr->points.size() << endl;
  // pcl::PCDWriter writer;
  // writer.write("/home/rick/Documents/a_system/src/pointcloud_processing/src/seam_cloud.pcd", *cloud_ptr, false) ;


}



























// PointCloud::Ptr show_grooveRegion_onProfile(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_seamRegion)
// {
//   PointCloud::Ptr grooveRegion_onProfile (new PointCloud);

//   // for(float i = 0; i < cloud_ptr->points.size(); i++)
//   // {
//   //   pcl::PointXYZRGB p;
    
//   //   p.x = cloud_ptr->points[ i ].x;
//   //   p.y = cloud_ptr->points[ i ].y;
//   //   p.z = cloud_ptr->points[ i ].z;
//   //   p.b = 200;
//   //   p.g = 200;
//   //   p.r = 200;

//   //   for(float j = 0; j < cloud_seamRegion->points.size(); j++)
//   //   {
//   //     if(fabs(cloud_ptr->points[i].x - cloud_seamRegion->points[j].x) < 1e-5 &&
//   //        fabs(cloud_ptr->points[i].y - cloud_seamRegion->points[j].y) < 1e-5 &&
//   //        fabs(cloud_ptr->points[i].z - cloud_seamRegion->points[j].z) < 1e-5)
//   //        {
//   //           p.x = cloud_ptr->points[ i ].x;
//   //           p.y = cloud_ptr->points[ i ].y;
//   //           p.z = cloud_ptr->points[ i ].z;
//   //           p.b = cloud_seamRegion->points[ j ].b;
//   //           p.g = cloud_seamRegion->points[ j ].g;
//   //           p.r = cloud_seamRegion->points[ j ].r;

//   //           break;
//   //        }
//   //   }

//   //   grooveRegion_onProfile->points.push_back( p ); 
//   // }

//   for(float i = 0; i < cloud_ptr->points.size(); i++)
//   {
//     pcl::PointXYZRGB p;

//     p.x = cloud_ptr->points[ i ].x;
//     p.y = cloud_ptr->points[ i ].y;
//     p.z = cloud_ptr->points[ i ].z;
//     p.b = 200;
//     p.g = 200;
//     p.r = 200;

//     grooveRegion_onProfile->points.push_back( p ); 
//   }

//   for(float j = 0; j < cloud_seamRegion->points.size(); j++)
//   {
//     pcl::PointXYZRGB p;

//     p.x = cloud_seamRegion->points[ j ].x;
//     p.y = cloud_seamRegion->points[ j ].y;
//     p.z = cloud_seamRegion->points[ j ].z;
//     p.b = cloud_seamRegion->points[ j ].b;
//     p.g = cloud_seamRegion->points[ j ].g;
//     p.r = cloud_seamRegion->points[ j ].r;

//     grooveRegion_onProfile->points.push_back( p ); 
//   }


//   cout << "grooveRegion_onProfile->points.size(): " << grooveRegion_onProfile->points.size() << endl;

//   return grooveRegion_onProfile;
// }


// vector< vector<int> > Segment_seam_region(PointCloud::Ptr cloud_seamRegion)
// {
//   float min_x = 0, max_x = 0;
//   float min_y = 0, max_y = 0;
//   float min_z = 0, max_z = 0;

//   // find maximum for XX YY ZZ
//   for(float i = 0; i < cloud_seamRegion->points.size(); i++)
//   { 
//     if(i == 0)
//     {
//       min_x = cloud_seamRegion->points[ i ].x;
//       min_y = cloud_seamRegion->points[ i ].y;
//       min_z = cloud_seamRegion->points[ i ].z;

//       max_x = cloud_seamRegion->points[ i ].x;
//       max_y = cloud_seamRegion->points[ i ].y;
//       max_z = cloud_seamRegion->points[ i ].z;
//     }
//     //x
//     if (max_x < cloud_seamRegion->points[ i ].x)
//     {
//       max_x = cloud_seamRegion->points[ i ].x;
//     }

//     if(min_x > cloud_seamRegion->points[ i ].x)
//     {
//       min_x = cloud_seamRegion->points[ i ].x;
//     }
//     //y
//     if (max_y < cloud_seamRegion->points[ i ].y)
//     {
//       max_y = cloud_seamRegion->points[ i ].y;
//     }

//     if(min_y > cloud_seamRegion->points[ i ].y)
//     {
//       min_y = cloud_seamRegion->points[ i ].y;
//     }
//     //z
//     if (max_z < cloud_seamRegion->points[ i ].z)
//     {
//       max_z = cloud_seamRegion->points[ i ].z;
//     }

//     if(min_z > cloud_seamRegion->points[ i ].z)
//     {
//       min_z = cloud_seamRegion->points[ i ].z;
//     }
//   }

//   float range_x = max_x - min_x, range_y = max_y - min_y, range_z = max_z - min_z;
//   cout << "max_x - min_x:  " << range_x << endl;
//   cout << "max_y - min_y:  " << range_y << endl;
//   cout << "max_z - min_z:  " << range_z << endl ;

//   // find largest range in X Y Z for defining main direction of seam pointcloud
//   char range_max = '0';
//   if( range_x > range_y)
//   {
//     range_max = 'x';
//   }
//   else
//   {
//     range_max = 'y';
//   }
  
//   if( range_max == range_x)
//   {
//     if( range_x > range_z)
//     {
//       range_max = 'x';
//     }
//     else
//     {
//       range_max = 'z';
//     }
//   }

//   if( range_max == range_y)
//   {
//     if( range_y > range_z)
//     {
//       range_max = 'y';
//     }
//     else
//     {
//       range_max = 'z';
//     }
//   }
//   cout << "range_max:  " << range_max << endl ;

//   //define segmentation number and compute interval length of each segment
//   float segmentation_count = 50, seg_interval = range_x / segmentation_count;

//   //define double verctor for recording segment point
//   vector< vector<int> > seg_pointcloud;
//   seg_pointcloud.resize(segmentation_count);

//   //get 50 segment with each point position
//   switch (range_max)
//   {
//     case 'x':
//       for( int j = 0; j < segmentation_count; j++)
//       {
//         for(float i = 0; i < cloud_seamRegion->points.size(); i++)
//         {
//           if(cloud_seamRegion->points[i].x >= min_x + seg_interval*(j) && cloud_seamRegion->points[i].x < seg_interval*(j+1) + min_x)
//           {
//             seg_pointcloud[j].push_back( i );
//           }
//           if(cloud_seamRegion->points[i].x == seg_interval*(segmentation_count) + min_x && j == segmentation_count - 1)
//           {
//             seg_pointcloud[segmentation_count - 1].push_back( i );            
//           }
//         }
//       } 
//       break;

//     case 'y':
//       for(float i = 0; i < cloud_seamRegion->points.size(); i++)
//       {
        
//       } 
//       break;

//     case 'z':
//       for(float i = 0; i < cloud_seamRegion->points.size(); i++)
//       {
        
//       } 
//       break;

//     default:
//       break;
//   }

//   //check total points of all the segment 
//   float seg_counttt = 0;
//   for( float j = 0; j < seg_pointcloud.size(); j++)
//   {
//     cout << "seg_pointcloud[j].size(): " << seg_pointcloud[j].size() << endl; 
//     seg_counttt += seg_pointcloud[j].size();
//   }

//   cout << "seg_counttt:" << seg_counttt << endl;
//   cout << "cloud_seamRegion->points.size(): " << cloud_seamRegion->points.size() << endl << endl;

//   //show segmentation divided into 3 types of color
//   for(int k = 0; k < seg_pointcloud.size(); k++)
//   {
//     for( float j = 0; j < seg_pointcloud[k].size(); j++)
//     {
//       switch (k % 3)
//       {
//         case 0:
//           cloud_seamRegion->points[ seg_pointcloud[k][j] ].b = 200;
//           cloud_seamRegion->points[ seg_pointcloud[k][j] ].g = 0;
//           cloud_seamRegion->points[ seg_pointcloud[k][j] ].r = 0;
//           break;
//         case 1:
//           cloud_seamRegion->points[ seg_pointcloud[k][j] ].b = 0;
//           cloud_seamRegion->points[ seg_pointcloud[k][j] ].g = 200;
//           cloud_seamRegion->points[ seg_pointcloud[k][j] ].r = 0;
//           break;
//         case 2:
//           cloud_seamRegion->points[ seg_pointcloud[k][j] ].b = 0;
//           cloud_seamRegion->points[ seg_pointcloud[k][j] ].g = 0;
//           cloud_seamRegion->points[ seg_pointcloud[k][j] ].r = 200;
//         break;
//       }
//     }
//   }
//   cout << "seg_pointcloud.size(): " << seg_pointcloud.size() << endl;
//   cout <<"segmentation is done!!!" << endl << endl;

//   return seg_pointcloud;
// }


// vector<float> cylinder_pathComputation(PointCloud::Ptr cloud_seamRegion, PointCloud::Ptr path_cloud);

// vector<float> Path_Generation(vector< vector<int> > seg_pointcloud, PointCloud::Ptr cloud_seamRegion, PointCloud::Ptr path_cloud, PointCloud::Ptr path_cloud_showRviz)
// {
//   pcl::PointXYZRGB Xp_goal;
//   Xp_goal.x = 0; Xp_goal.y = 0; Xp_goal.z = 0;

//   pcl::PointXYZ Xp;
//   Xp.x = 0; Xp.y = 0; Xp.z = 0;

//   pcl::PointXYZ Xpi; 
//   Xpi.x = 0; Xpi.y = 0; Xpi.z = 0;

//   float theta = 0.01;
//   float loop_max = 10000;
//   float epsilon = 0.0001;
//   float loss = 0;

//   //定义每个segmet的点云
//   Cloud::Ptr cloud_segment (new Cloud);

//   for(int k = 0; k < seg_pointcloud.size(); k++)
//   {
//     //迭代计算每个segment 的中心点及其position
//     for (float loop_count = 0; loop_count < loop_max; loop_count++)
//     {
//       // cost computation:
//       float cost_function = 0;
//       for( float j = 0; j < seg_pointcloud[k].size(); j++)
//       {
//         cost_function += sqrt(pow(Xp.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
//                               pow(Xp.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
//                               pow(Xp.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );
//       }

//       //gradient computation:
//       pcl::PointXYZ Gradient_Xp;
//       for( float j = 0; j < seg_pointcloud[k].size(); j++)
//       {
//         Gradient_Xp.x += (Xp.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x) / 
//                           sqrt(pow(Xp.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
//                                pow(Xp.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
//                                pow(Xp.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );

//         Gradient_Xp.y += (Xp.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y) / 
//                           sqrt(pow(Xp.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
//                                pow(Xp.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
//                                pow(Xp.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );

//         Gradient_Xp.z += (Xp.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z) / 
//                           sqrt(pow(Xp.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
//                                pow(Xp.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
//                                pow(Xp.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );
//       }

//       // xyz step in one step
//       Xpi.x = Xp.x - theta * Gradient_Xp.x;
//       Xpi.y = Xp.y - theta * Gradient_Xp.y;
//       Xpi.z = Xp.z - theta * Gradient_Xp.z;

//       //compute updated cost function
//       float cost_function_update = 0;
//       for( float j = 0; j < seg_pointcloud[k].size(); j++)
//       {
//         cost_function_update += sqrt(pow(Xpi.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
//                                      pow(Xpi.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
//                                      pow(Xpi.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );
//       }

//       //判断loss是否小于阈值
//       loss = cost_function - cost_function_update;
//       // cout << "loss: " << loss << endl;

//       //loss 大于 最小误差， 继续迭代
//       if(loss > epsilon)
//       {
//         Xp.x = Xpi.x;
//         Xp.y = Xpi.y;
//         Xp.z = Xpi.z;

//         cost_function = cost_function_update;
//       }

//       //减少步长
//       else if(cost_function_update - cost_function > epsilon)
//       {
//         theta = theta * 0.8;
//       }

//       //满足迭代条件，得到目标点
//       else
//       {
//         Xp_goal.x = Xp.x;
//         Xp_goal.y = Xp.y;
//         Xp_goal.z = Xp.z;
//         Xp_goal.b = 0;
//         Xp_goal.g = 0;
//         Xp_goal.r = 0;
//         cloud_seamRegion->points.push_back( Xp_goal ) ; 

//         break;
//       }
//     }

//     // cout << "seg_count: " << k+1 << endl;
//     // cout << "Xp_goal: " << Xp_goal << endl << endl;
//     path_cloud->points.push_back( Xp_goal );
//     path_cloud_showRviz->points.push_back( Xp_goal );

//     // 插值， 构成完整轨迹, 每个path点之间插5个点
//     if(k > 0)
//     {
//       pcl::PointXYZRGB p_add1, p_add2, p_add3, p_add4, p_add5, p_add6, p_add7;

//       p_add1.x = (path_cloud->points[k - 1].x + path_cloud->points[k].x) / 2.0;
//       p_add1.y = (path_cloud->points[k - 1].y + path_cloud->points[k].y) / 2.0;
//       p_add1.z = (path_cloud->points[k - 1].z + path_cloud->points[k].z) / 2.0;
//       cloud_seamRegion->points.push_back( p_add1 );
//       path_cloud_showRviz->points.push_back( p_add1 );

//       p_add2.x = (path_cloud->points[k - 1].x + p_add1.x) / 2.0;
//       p_add2.y = (path_cloud->points[k - 1].y + p_add1.y) / 2.0;
//       p_add2.z = (path_cloud->points[k - 1].z + p_add1.z) / 2.0;
//       cloud_seamRegion->points.push_back( p_add2 );
//       path_cloud_showRviz->points.push_back( p_add2 );

//       p_add3.x = (p_add1.x + path_cloud->points[k].x) / 2.0;
//       p_add3.y = (p_add1.y + path_cloud->points[k].y) / 2.0;
//       p_add3.z = (p_add1.z + path_cloud->points[k].z) / 2.0;
//       cloud_seamRegion->points.push_back( p_add3 );
//       path_cloud_showRviz->points.push_back( p_add3 );

//       p_add4.x = (path_cloud->points[k - 1].x + p_add2.x) / 2.0;
//       p_add4.y = (path_cloud->points[k - 1].y + p_add2.y) / 2.0;
//       p_add4.z = (path_cloud->points[k - 1].z + p_add2.z) / 2.0;
//       cloud_seamRegion->points.push_back( p_add4 );
//       path_cloud_showRviz->points.push_back( p_add4 );

//       p_add5.x = (p_add1.x + p_add2.x) / 2.0;
//       p_add5.y = (p_add1.y + p_add2.y) / 2.0;
//       p_add5.z = (p_add1.z + p_add2.z) / 2.0;
//       cloud_seamRegion->points.push_back( p_add5 );
//       path_cloud_showRviz->points.push_back( p_add5 );

//       p_add6.x = (p_add1.x + p_add3.x) / 2.0;
//       p_add6.y = (p_add1.y + p_add3.y) / 2.0;
//       p_add6.z = (p_add1.z + p_add3.z) / 2.0;
//       cloud_seamRegion->points.push_back( p_add6 );
//       path_cloud_showRviz->points.push_back( p_add6 );

//       p_add7.x = (path_cloud->points[k].x + p_add3.x) / 2.0;
//       p_add7.y = (path_cloud->points[k].y + p_add3.y) / 2.0;
//       p_add7.z = (path_cloud->points[k].z + p_add3.z) / 2.0;
//       cloud_seamRegion->points.push_back( p_add7 );
//       path_cloud_showRviz->points.push_back( p_add7 );
//     }
//   }
 
//   cout << "path_cloud->points.size(): "          << path_cloud->points.size() << endl;
//   cout << "path_cloud_showRviz->points.size(): " << path_cloud_showRviz->points.size() << endl << endl;


//   /////////////////////////////////////////////////////////////////////////////////////////////////////////////

//   vector<float> orientation_pathpoints = cylinder_pathComputation(cloud_seamRegion, path_cloud);

//   return orientation_pathpoints;
// }

// Point3f circle_estimation(vector<Point3f> path_points);

// vector<float> cylinder_pathComputation(PointCloud::Ptr cloud_seamRegion, PointCloud::Ptr path_cloud)
// {
//   vector<Point3f> path_points;
//   for( float i = 0; i < path_cloud->points.size(); i++)
//   {
//     Point3f p;

//     p.x = path_cloud->points[ i ].x;
//     p.y = path_cloud->points[ i ].y;
//     p.z = path_cloud->points[ i ].z;

//     path_points.push_back( p );
//   }
  
//   path_cloud->clear();
//   cout << "path_points.size(): "          << path_points.size() << endl;

//   Point3f p_circle = circle_estimation(path_points);

//   pcl::PointXYZRGB p ;
//   p.x = p_circle.x;
//   p.y = p_circle.y;
//   p.z = p_circle.z;
//   p.b = 0;
//   p.g = 0;
//   p.r = 200;
//   cloud_seamRegion->points.push_back(p);


//   vector<Point3f> cylinder_path_points;        
//   Point3f path_cylinder;

//   vector<float> orientation_pathpoints;
//   float   k_orientation;

//   for(int i = 0; i < path_points.size(); i++)
//   {
//     k_orientation = (p_circle.y - path_points[i].y) / (p_circle.x - path_points[i].x);
//     cout << "yaw: " << atan( -1 / k_orientation ) * 180 / M_PI << endl;
//     orientation_pathpoints.push_back( atan( -1 / k_orientation ) * 180 / M_PI );

//     //////////////////////////////////////////////////////

//     path_cylinder.x = path_points[i].x;//(2 * path_points[i].x) - p_circle.x;
//     path_cylinder.y = path_points[i].y;//(2 * path_points[i].y) - p_circle.y;
//     path_cylinder.z = path_points[i].z;//(2 * path_points[i].z) - p_circle.z;
//     cout << "path_cylinder: " << path_cylinder << endl;

//     cylinder_path_points.push_back( path_cylinder );

//     // pcl::PointXYZRGB p ;
//     // p.x = path_cylinder.x;
//     // p.y = path_cylinder.y;
//     // p.z = path_cylinder.z;
//     // p.b = 0;
//     // p.g = 0;
//     // p.r = 200;
//     // cloud_seamRegion->points.push_back(p);
//   }
//   cout << "orientation_pathpoints.size(): " << orientation_pathpoints.size() << endl;
//   cout << "cylinder_path_points.size(): "      << cylinder_path_points.size() << endl;

//   for(int i = 0; i < cylinder_path_points.size(); i++)
//   {
//     pcl::PointXYZRGB p;

//     p.x = cylinder_path_points[i].x;
//     p.y = cylinder_path_points[i].y;
//     p.z = cylinder_path_points[i].z;

//     path_cloud->points.push_back(p);
//   }

//   cout << "path_cloud->points.size(): "        << path_cloud->points.size() << endl;

//   return  orientation_pathpoints;
// }

// Point3f circle_estimation(vector<Point3f> path_points)
// {
//   Point3f p_start, p_mid, p_end;

//   //赋值给三个点
//   p_start.x = path_points[0].x;                      p_start.y = path_points[0].y;                      p_start.z = path_points[0].z;
//     p_mid.x = path_points[path_points.size() / 2].x;   p_mid.y = path_points[path_points.size() / 2].y;   p_mid.z = path_points[path_points.size() / 2].z;
//     p_end.x = path_points[path_points.size() - 1].x;   p_end.y = path_points[path_points.size() - 1].y;   p_end.z = path_points[path_points.size() - 1].z;

//   //算两条线的斜率
//   float line1_k = (p_mid.y - p_start.y) / (p_mid.x - p_start.x);
//   float line2_k = (p_mid.y -   p_end.y) / (p_mid.x -   p_end.x);

//   //得到两条线直线方程
//   cout << "line1_k: " << line1_k << endl;
//   cout << "line2_k: " << line2_k << endl;
//   // cout << "得到两条直线方程为：" << endl;
//   cout << "y = " << line1_k << " * (x - p_mid.x) + p_mid.y: " << endl;
//   cout << "y = " << line2_k << " * (x - p_mid.x) + p_mid.y: " << endl;

//   //求两对点的中间点
//   Point3f p_midpoint1, p_midpoint2;

//   p_midpoint1.x = (p_start.x + p_mid.x) / 2; p_midpoint1.y = (p_start.y + p_mid.y) / 2; p_midpoint1.z = (p_start.z + p_mid.z) / 2;
//   p_midpoint2.x = (  p_end.x + p_mid.x) / 2; p_midpoint2.y = (  p_end.y + p_mid.y) / 2; p_midpoint2.z = (  p_end.z + p_mid.z) / 2;
//   // cout << "得到两中点为：" << endl;
//   cout << "p_midpoint1：" << p_midpoint1 << endl;
//   cout << "p_midpoint2：" << p_midpoint2 << endl;
//   // cout << "相垂直的两条线斜率为：" << endl;
//   cout << "-1 / line1_k:" << -1 / line1_k << endl;
//   cout << "-1 / line2_k:" << -1 / line2_k << endl;
//   // cout << "得到两条垂直直线方程为：" << endl;
//   cout << "y = " <<  -1 / line1_k  << " * (x - " << p_midpoint1.x << " ) + " << p_midpoint1.y << endl;
//   cout << "y = " <<  -1 / line2_k  << " * (x - " << p_midpoint2.x << " ) + " << p_midpoint2.y << endl;

//   //求估计的圆心：
//   Point3f p_circle;

//   p_circle.x = ( ((-1 / line1_k) * p_midpoint1.x - p_midpoint1.y) - ((-1 / line2_k) * p_midpoint2.x - p_midpoint2.y) ) / ( (-1 / line1_k) - (-1 / line2_k) );
//   p_circle.y = (-1 / line1_k) * (p_circle.x - p_midpoint1.x) + p_midpoint1.y; 

//   for(int i = 0; i < path_points.size(); i++)
//   {
//     p_circle.z += path_points[i].z;
//   }
//   p_circle.z = p_circle.z / path_points.size();

//   cout << "最后估计的圆中心为：" << endl;
//   cout << "p_circle: " << p_circle << endl << endl;

//   return p_circle;
// }


 