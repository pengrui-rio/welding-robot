#include <algorithm.h>


Cloud::Ptr read_pointcloud (PointCloud::Ptr cloud_ptr_show)
{
  //seam detection
  Cloud::Ptr cloud_ptr (new Cloud);

  // PCD reader
  pcl::PCDReader reader;
  reader.read("/home/robot/Documents/a_system/src/seam_detection/save_pcd/bottom_straight.pcd", *cloud_ptr);
  
  cout << "PointCLoud size() " << cloud_ptr->width * cloud_ptr->height
       << " data points " << pcl::getFieldsList (*cloud_ptr) << "." << endl << endl;


  //bottom_straight
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZRGB p;

    p.x = cloud_ptr->points[i].x + 0.05; 
    p.y = cloud_ptr->points[i].y - 0.03;
    p.z = cloud_ptr->points[i].z - 0.02;//- 0.457; // 0.125
    p.b = 200; 
    p.g = 200;
    p.r = 200;
    cloud_ptr_show->points.push_back( p );    
  }


  //box
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZRGB p;

    p.x = cloud_ptr->points[i].x + 0.03; 
    p.y = cloud_ptr->points[i].y - 0.03;
    p.z = cloud_ptr->points[i].z ;//- 0.457; // 0.125
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


  cout << "cloud_ptr_show->points.size()" << cloud_ptr->points.size() << endl;

  return cloud_ptr;
}


vector<Point3f> allPoint_normal_computation(float sphere_computation, Cloud::Ptr cloud_ptr )
{
  //define kdtree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  Normal::Ptr cloud_normals_ptr (new Normal);
  Normal& cloud_normals = *cloud_normals_ptr;

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_ptr);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (sphere_computation);
  ne.compute (cloud_normals); // get cloud normals

  cout << "cloud_normals.size(): " << cloud_normals.size() << endl;

  // define unit normals for each point
  vector<Point3f> unit_normals;

  float M = 0; // M = sqrt(x^2 + y^2 + z^2)

  // compute unit normals for each point
  for(float i = 0; i < cloud_normals.size(); i++)
  { 
    Point3f p ;

    M = sqrt(pow(cloud_normals[i].normal_x, 2) + pow(cloud_normals[i].normal_y, 2) + pow(cloud_normals[i].normal_z, 2));

    p.x = cloud_normals[i].normal_x / M;
    if(p.x < 0) 
    {
      p.x = -p.x;
    }
    p.y = cloud_normals[i].normal_y / M;
    if(p.y < 0) 
    {
      p.y = -p.y;
    }
    p.z = cloud_normals[i].normal_z / M;
    if(p.z < 0) 
    {
      p.z = -p.z;
    }
    unit_normals.push_back( p );
  }

  cout << "unit_normals.size(): " << unit_normals.size() << endl << endl;

  return unit_normals;
}


void basic_normal_computation(Cloud::Ptr cloud_ptr, vector<Point3f> cloud_normals, float *basic_normal_x, float *basic_normal_y, float *basic_normal_z)
{
  float total_pointcount = 0;

  float sum_normal_x = 0, sum_normal_y = 0, sum_normal_z = 0;

  // compute main direction normal
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  { 
    if ( __isnan(cloud_normals[i].x) == true || __isnan(cloud_normals[i].y) == true || __isnan(cloud_normals[i].z) == true)
    {
      continue;
    }
    total_pointcount++;

    sum_normal_x += cloud_normals[i].x;
    sum_normal_y += cloud_normals[i].y;
    sum_normal_z += cloud_normals[i].z;
  }

  float M = sqrt(pow(sum_normal_x, 2) + pow(sum_normal_y, 2) + pow(sum_normal_z, 2)); // M = sqrt(x^2 + y^2 + z^2)

  *basic_normal_x = 1.0 * sum_normal_x / M;
  *basic_normal_y = 1.0 * sum_normal_y / M;
  *basic_normal_z = 1.0 * sum_normal_z / M;

  cout << "initial points.size(): " << cloud_ptr->points.size() << endl;
  cout << "nan-point count: "       << cloud_ptr->points.size() - total_pointcount << endl; 
  cout << "basic_normal_x: "        << *basic_normal_x << endl;
  cout << "basic_normal_y: "        << *basic_normal_y << endl;
  cout << "basic_normal_z: "        << *basic_normal_z << endl << endl;
}

 
vector<float> Point_descriptor_computation(PointCloud::Ptr descriptor_cloud, Cloud::Ptr cloud_ptr, vector<Point3f> cloud_normals, float basic_normal_x, float basic_normal_y, float basic_normal_z)
{
  // define vector for descriptor
  vector<float> Dir_descriptor ;

  float dir_descriptor = 0;

  // compute descriptor    two vectors    COS_ab = a*b / (|a| * |b|) 
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  { 
  //   float a_b = cloud_normals[ i ].x * basic_normal_x +
  //               cloud_normals[ i ].y * basic_normal_y +
  //               cloud_normals[ i ].z * basic_normal_z ;  

  //   float a2 = sqrt(pow(cloud_normals[ i ].x, 2) +
  //                   pow(cloud_normals[ i ].y, 2) +
  //                   pow(cloud_normals[ i ].z, 2));  

  //   float b2 = sqrt(pow(basic_normal_x, 2) +
  //                   pow(basic_normal_y, 2) +
  //                   pow(basic_normal_z, 2));  

  //   float COS_ab = a_b / (a2 * b2) ;

  //   float theta = acos( COS_ab ) * 180.0 / M_PI ;

  //   if( __isnan(theta) == true )
  //   {
  //     continue;
  //   }
    
    float COS_ab = sqrt(pow(cloud_normals[ i ].x - basic_normal_x, 2) + 
                        pow(cloud_normals[ i ].y - basic_normal_y, 2) + 
                        pow(cloud_normals[ i ].z - basic_normal_z, 2));

    Dir_descriptor.push_back( COS_ab );     

    pcl::PointXYZRGB p;
    p.x = cloud_ptr->points[i].x; 
    p.y = cloud_ptr->points[i].y;
    p.z = cloud_ptr->points[i].z;
    p.b = 0; 
    p.g = 0;
    p.r = 0;

    // build up descriptor_cloud
    descriptor_cloud->points.push_back( p );        
  }

  // find Dir_descriptor_max + Dir_descriptor_min
  float Dir_descriptor_max = 0, Dir_descriptor_min = 0;
  for(float i = 0; i < descriptor_cloud->points.size(); i++)
  { 
    if(i == 0)
    {
      Dir_descriptor_max = Dir_descriptor[0];
      Dir_descriptor_min = Dir_descriptor[0];
    }

    if (Dir_descriptor_max < Dir_descriptor[i])
    {
      Dir_descriptor_max = Dir_descriptor[i];
    }

    if(Dir_descriptor_min > Dir_descriptor[i])
    {
      Dir_descriptor_min = Dir_descriptor[i];
    }
  }

  cout << "Dir_descriptor_max: "    << Dir_descriptor_max << endl;
  cout << "Dir_descriptor_min: "    << Dir_descriptor_min << endl;

  cout << "Dir_descriptor size(): " << Dir_descriptor.size()       << endl;
  cout << "descriptor_cloud size(): " << descriptor_cloud->points.size() << endl << endl;

  // use blue + red to indicate distribution of descriptor
  float weight_of_descriptor = 0;
  for(float i = 0; i < descriptor_cloud->points.size(); i++)
  { 
    weight_of_descriptor = (Dir_descriptor[i] - Dir_descriptor_min) / (Dir_descriptor_max - Dir_descriptor_min);

    descriptor_cloud->points[i].b = 255 * weight_of_descriptor;
    descriptor_cloud->points[i].g = 0;
    descriptor_cloud->points[i].r = 255 * (1 - weight_of_descriptor);
  }

  return Dir_descriptor;
}



vector<float> Point_variance_computation(Cloud::Ptr cloud_tree_variance, PointCloud::Ptr cloud_tree_variance_show, PointCloud::Ptr descriptor_cloud, vector<float> Dir_descriptor)
{
  // compute cloud_tree_variance
  for(float i = 0; i < descriptor_cloud->points.size(); i++)
  { 
    pcl::PointXYZ p;

    p.x = descriptor_cloud->points[i].x;
    p.y = descriptor_cloud->points[i].y;
    p.z = descriptor_cloud->points[i].z;

    cloud_tree_variance->points.push_back( p );
  }

  // define kdtree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  // 创建一个 KdTree 对象
  kdtree.setInputCloud (cloud_tree_variance);  // 将前面创建的随机点云作为 KdTree 输入
  vector<int> pointIdxRadiusSearch; // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  vector<float> pointRadiusSquaredDistance;
  float radius = 0.005;

  //define variance_descriptor
  vector<float> variance_descriptor ;

  // compute variance in neiborhood for each point
  for(float i = 0; i < cloud_tree_variance->points.size(); i++)
  { 
    kdtree.radiusSearch (cloud_tree_variance->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);  

    float variance = 0, sum = 0, average_kdtree = 0;

    for (float j = 0; j < pointIdxRadiusSearch.size(); j++)
    {
      sum += Dir_descriptor[ pointIdxRadiusSearch[j] ];
    }
    average_kdtree = sum / pointIdxRadiusSearch.size();

    for (float j = 0; j < pointIdxRadiusSearch.size(); j++)
    {
      variance += pow(Dir_descriptor[pointIdxRadiusSearch[j]] - average_kdtree, 2);
    }    

    variance = variance / pointIdxRadiusSearch.size();
    variance_descriptor.push_back( variance );
  }

  float Var_descriptor_min = 0, Var_descriptor_max = 0;

  // find Var_descriptor_min + Var_descriptor_max
  for(float i = 0; i < variance_descriptor.size(); i++)
  { 
    if(i == 0)
    {
      Var_descriptor_max = variance_descriptor[0];
      Var_descriptor_min = variance_descriptor[0];
    }

    if (Var_descriptor_max < variance_descriptor[i])
    {
      Var_descriptor_max = variance_descriptor[i];
    }

    if(Var_descriptor_min > variance_descriptor[i])
    {
      Var_descriptor_min = variance_descriptor[i];
    }
  }
  cout << "Var_descriptor_max: "    << Var_descriptor_max << endl;
  cout << "Var_descriptor_min: "    << Var_descriptor_min << endl;

  cout << "cloud_tree_variance->points.size(): " << cloud_tree_variance->points.size() << endl;
  cout << "variance_descriptor.size(): " << variance_descriptor.size() << endl << endl;

  //define weight_variance_threshold
  float weight_variance_threshold = (Var_descriptor_max - Var_descriptor_min) / 3.5;

  //use weight_variance_threshold to separate target region
  for(float i = 0; i < cloud_tree_variance->points.size(); i++)
  { 
    pcl::PointXYZRGB p;
    p.x = cloud_tree_variance->points[i].x; 
    p.y = cloud_tree_variance->points[i].y;
    p.z = cloud_tree_variance->points[i].z;

    float weight_variance_descriptor = (variance_descriptor[i] - Var_descriptor_min) / (Var_descriptor_max - Var_descriptor_min);

    if( weight_variance_descriptor > weight_variance_threshold)
    {
      p.b = 200;
      p.g = 0;
      p.r = 0;
    }
    else
    {
      p.b = 200;
      p.g = 200;
      p.r = 200;
    }
    cloud_tree_variance_show->points.push_back( p );    
  }

  return variance_descriptor;
}




void exact_Target_regionPointcloud(PointCloud::Ptr cloud_tree_rm_irrelativePoint, Cloud::Ptr cloud_tree_variance, PointCloud::Ptr cloud_tree_variance_show)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_sreenout;  // 将前面创建的随机点云作为 KdTree 输入
  kdtree_sreenout.setInputCloud (cloud_tree_variance);
  vector<int> pointIdxRadiusSearch_sreenout;         // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  vector<float> pointRadiusSquaredDistance_sreenout;
  float radius_sreenout = 0.005;  // 指定随机半径

  // if blue points number in neiborhood beyonds 50% , add the point into cloud_tree_rm_irrelativePoint
  for(float i = 0; i < cloud_tree_variance_show->points.size(); i++)
  { 
    pcl::PointXYZRGB p;

    kdtree_sreenout.radiusSearch(cloud_tree_variance->points[i], radius_sreenout, pointIdxRadiusSearch_sreenout, pointRadiusSquaredDistance_sreenout);  

    float blue_count = 0;
    for (float j = 0; j < pointIdxRadiusSearch_sreenout.size(); j++)
    {
      if (cloud_tree_variance_show->points[ pointIdxRadiusSearch_sreenout[j] ].b == 200 &&
          cloud_tree_variance_show->points[ pointIdxRadiusSearch_sreenout[j] ].g == 0   &&
          cloud_tree_variance_show->points[ pointIdxRadiusSearch_sreenout[j] ].r == 0)
          {
            blue_count++;
          }
    }

    // > 50%    set the size of neiborhood > 1
    if (1.0 * blue_count / pointIdxRadiusSearch_sreenout.size() > 0.5 && pointIdxRadiusSearch_sreenout.size() > 1)
    {
      p.x = cloud_tree_variance_show->points[ i ].x;
      p.y = cloud_tree_variance_show->points[ i ].y;
      p.z = cloud_tree_variance_show->points[ i ].z;
      p.b = 200;
      p.g = 0;
      p.r = 0;

      cloud_tree_rm_irrelativePoint->points.push_back( p );
    }

  }
  cout << "cloud_tree_variance_show->points.size(): " << cloud_tree_variance_show->points.size() << endl;
  cout << "cloud_tree_rm_irrelativePoint->points.size(): " << cloud_tree_rm_irrelativePoint->points.size() << endl << endl;
}



void Exact_seam_region(PointCloud::Ptr cloud_tree_rm_irrelativePoint, PointCloud::Ptr cloud_seamRegion)
{
  // push cloud_tree_rm_irrelativePoint into ec_tree_cloud
  Cloud::Ptr ec_tree_cloud (new Cloud);
  for(float i = 0; i < cloud_tree_rm_irrelativePoint->points.size(); i++)
  { 
    pcl::PointXYZ p;

    p.x = cloud_tree_rm_irrelativePoint->points[ i ].x;
    p.y = cloud_tree_rm_irrelativePoint->points[ i ].y;
    p.z = cloud_tree_rm_irrelativePoint->points[ i ].z;

    ec_tree_cloud->points.push_back( p );
  }

  //define kdtree for euclidean clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr ec_tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ec_tree->setInputCloud (ec_tree_cloud);//创建点云索引向量，用于存储实际的点云信息

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> EC;

  EC.setClusterTolerance (0.005); //设置近邻搜索的搜索半径为2cm
  EC.setMinClusterSize (1);//设置一个聚类需要的最少点数目为100
  EC.setMaxClusterSize (10000000); //设置一个聚类需要的最大点数目为25000
  EC.setSearchMethod (ec_tree);//设置点云的搜索机制
  EC.setInputCloud (ec_tree_cloud);// input cloud
  EC.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中
  
  cout << "ec_tree_cloud->points.size(): "  << ec_tree_cloud->points.size() << endl ;
  cout << "cluster_indices.size(): " << cluster_indices.size() << endl;

  // put largest cluster into final seam cloud
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointXYZRGB p;

    Cloud::Ptr cloud_cluster (new Cloud);
    
    // push back
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (ec_tree_cloud->points[*pit]);  
    }

    // put point into cloud_seamRegion
    for(float i = 0; i < cloud_cluster->points.size(); i++)
    { 
      p.x = cloud_cluster->points[ i ].x;
      p.y = cloud_cluster->points[ i ].y;
      p.z = cloud_cluster->points[ i ].z;
      p.b = 200;
      p.g = 0;
      p.r = 0;

      cloud_seamRegion->points.push_back( p ); 
    }

    break;
  }
  cout << "cloud_seamRegion->points.size(): " << cloud_seamRegion->points.size() << endl << endl;
}

PointCloud::Ptr show_grooveRegion_onProfile(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_seamRegion)
{
  PointCloud::Ptr grooveRegion_onProfile (new PointCloud);

  // for(float i = 0; i < cloud_ptr->points.size(); i++)
  // {
  //   pcl::PointXYZRGB p;
    
  //   p.x = cloud_ptr->points[ i ].x;
  //   p.y = cloud_ptr->points[ i ].y;
  //   p.z = cloud_ptr->points[ i ].z;
  //   p.b = 200;
  //   p.g = 200;
  //   p.r = 200;

  //   for(float j = 0; j < cloud_seamRegion->points.size(); j++)
  //   {
  //     if(fabs(cloud_ptr->points[i].x - cloud_seamRegion->points[j].x) < 1e-5 &&
  //        fabs(cloud_ptr->points[i].y - cloud_seamRegion->points[j].y) < 1e-5 &&
  //        fabs(cloud_ptr->points[i].z - cloud_seamRegion->points[j].z) < 1e-5)
  //        {
  //           p.x = cloud_ptr->points[ i ].x;
  //           p.y = cloud_ptr->points[ i ].y;
  //           p.z = cloud_ptr->points[ i ].z;
  //           p.b = cloud_seamRegion->points[ j ].b;
  //           p.g = cloud_seamRegion->points[ j ].g;
  //           p.r = cloud_seamRegion->points[ j ].r;

  //           break;
  //        }
  //   }

  //   grooveRegion_onProfile->points.push_back( p ); 
  // }

  for(float i = 0; i < cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZRGB p;

    p.x = cloud_ptr->points[ i ].x;
    p.y = cloud_ptr->points[ i ].y;
    p.z = cloud_ptr->points[ i ].z;
    p.b = 200;
    p.g = 200;
    p.r = 200;

    grooveRegion_onProfile->points.push_back( p ); 
  }

  for(float j = 0; j < cloud_seamRegion->points.size(); j++)
  {
    pcl::PointXYZRGB p;

    p.x = cloud_seamRegion->points[ j ].x;
    p.y = cloud_seamRegion->points[ j ].y;
    p.z = cloud_seamRegion->points[ j ].z;
    p.b = cloud_seamRegion->points[ j ].b;
    p.g = cloud_seamRegion->points[ j ].g;
    p.r = cloud_seamRegion->points[ j ].r;

    grooveRegion_onProfile->points.push_back( p ); 
  }


  cout << "grooveRegion_onProfile->points.size(): " << grooveRegion_onProfile->points.size() << endl;

  return grooveRegion_onProfile;
}


vector< vector<int> > Segment_seam_region(PointCloud::Ptr cloud_seamRegion)
{
  float min_x = 0, max_x = 0;
  float min_y = 0, max_y = 0;
  float min_z = 0, max_z = 0;

  // find maximum for XX YY ZZ
  for(float i = 0; i < cloud_seamRegion->points.size(); i++)
  { 
    if(i == 0)
    {
      min_x = cloud_seamRegion->points[ i ].x;
      min_y = cloud_seamRegion->points[ i ].y;
      min_z = cloud_seamRegion->points[ i ].z;

      max_x = cloud_seamRegion->points[ i ].x;
      max_y = cloud_seamRegion->points[ i ].y;
      max_z = cloud_seamRegion->points[ i ].z;
    }
    //x
    if (max_x < cloud_seamRegion->points[ i ].x)
    {
      max_x = cloud_seamRegion->points[ i ].x;
    }

    if(min_x > cloud_seamRegion->points[ i ].x)
    {
      min_x = cloud_seamRegion->points[ i ].x;
    }
    //y
    if (max_y < cloud_seamRegion->points[ i ].y)
    {
      max_y = cloud_seamRegion->points[ i ].y;
    }

    if(min_y > cloud_seamRegion->points[ i ].y)
    {
      min_y = cloud_seamRegion->points[ i ].y;
    }
    //z
    if (max_z < cloud_seamRegion->points[ i ].z)
    {
      max_z = cloud_seamRegion->points[ i ].z;
    }

    if(min_z > cloud_seamRegion->points[ i ].z)
    {
      min_z = cloud_seamRegion->points[ i ].z;
    }
  }

  float range_x = max_x - min_x, range_y = max_y - min_y, range_z = max_z - min_z;
  cout << "max_x - min_x:  " << range_x << endl;
  cout << "max_y - min_y:  " << range_y << endl;
  cout << "max_z - min_z:  " << range_z << endl ;

  // find largest range in X Y Z for defining main direction of seam pointcloud
  char range_max = '0';
  if( range_x > range_y)
  {
    range_max = 'x';
  }
  else
  {
    range_max = 'y';
  }
  
  if( range_max == range_x)
  {
    if( range_x > range_z)
    {
      range_max = 'x';
    }
    else
    {
      range_max = 'z';
    }
  }

  if( range_max == range_y)
  {
    if( range_y > range_z)
    {
      range_max = 'y';
    }
    else
    {
      range_max = 'z';
    }
  }
  cout << "range_max:  " << range_max << endl ;

  //define segmentation number and compute interval length of each segment
  float segmentation_count = 50, seg_interval = range_x / segmentation_count;

  //define double verctor for recording segment point
  vector< vector<int> > seg_pointcloud;
  seg_pointcloud.resize(segmentation_count);

  //get 50 segment with each point position
  switch (range_max)
  {
    case 'x':
      for( int j = 0; j < segmentation_count; j++)
      {
        for(float i = 0; i < cloud_seamRegion->points.size(); i++)
        {
          if(cloud_seamRegion->points[i].x >= min_x + seg_interval*(j) && cloud_seamRegion->points[i].x < seg_interval*(j+1) + min_x)
          {
            seg_pointcloud[j].push_back( i );
          }
          if(cloud_seamRegion->points[i].x == seg_interval*(segmentation_count) + min_x && j == segmentation_count - 1)
          {
            seg_pointcloud[segmentation_count - 1].push_back( i );            
          }
        }
      } 
      break;

    case 'y':
      for(float i = 0; i < cloud_seamRegion->points.size(); i++)
      {
        
      } 
      break;

    case 'z':
      for(float i = 0; i < cloud_seamRegion->points.size(); i++)
      {
        
      } 
      break;

    default:
      break;
  }

  //check total points of all the segment 
  float seg_counttt = 0;
  for( float j = 0; j < seg_pointcloud.size(); j++)
  {
    cout << "seg_pointcloud[j].size(): " << seg_pointcloud[j].size() << endl; 
    seg_counttt += seg_pointcloud[j].size();
  }

  cout << "seg_counttt:" << seg_counttt << endl;
  cout << "cloud_seamRegion->points.size(): " << cloud_seamRegion->points.size() << endl << endl;

  //show segmentation divided into 3 types of color
  for(int k = 0; k < seg_pointcloud.size(); k++)
  {
    for( float j = 0; j < seg_pointcloud[k].size(); j++)
    {
      switch (k % 3)
      {
        case 0:
          cloud_seamRegion->points[ seg_pointcloud[k][j] ].b = 200;
          cloud_seamRegion->points[ seg_pointcloud[k][j] ].g = 0;
          cloud_seamRegion->points[ seg_pointcloud[k][j] ].r = 0;
          break;
        case 1:
          cloud_seamRegion->points[ seg_pointcloud[k][j] ].b = 0;
          cloud_seamRegion->points[ seg_pointcloud[k][j] ].g = 200;
          cloud_seamRegion->points[ seg_pointcloud[k][j] ].r = 0;
          break;
        case 2:
          cloud_seamRegion->points[ seg_pointcloud[k][j] ].b = 0;
          cloud_seamRegion->points[ seg_pointcloud[k][j] ].g = 0;
          cloud_seamRegion->points[ seg_pointcloud[k][j] ].r = 200;
        break;
      }
    }
  }
  cout << "seg_pointcloud.size(): " << seg_pointcloud.size() << endl;
  cout <<"segmentation is done!!!" << endl << endl;

  return seg_pointcloud;
}


vector<float> cylinder_pathComputation(PointCloud::Ptr cloud_seamRegion, PointCloud::Ptr path_cloud);

vector<float> Path_Generation(vector< vector<int> > seg_pointcloud, PointCloud::Ptr cloud_seamRegion, PointCloud::Ptr path_cloud, PointCloud::Ptr path_cloud_showRviz)
{
  pcl::PointXYZRGB Xp_goal;
  Xp_goal.x = 0; Xp_goal.y = 0; Xp_goal.z = 0;

  pcl::PointXYZ Xp;
  Xp.x = 0; Xp.y = 0; Xp.z = 0;

  pcl::PointXYZ Xpi; 
  Xpi.x = 0; Xpi.y = 0; Xpi.z = 0;

  float theta = 0.01;
  float loop_max = 10000;
  float epsilon = 0.0001;
  float loss = 0;

  //定义每个segmet的点云
  Cloud::Ptr cloud_segment (new Cloud);

  for(int k = 0; k < seg_pointcloud.size(); k++)
  {
    //迭代计算每个segment 的中心点及其position
    for (float loop_count = 0; loop_count < loop_max; loop_count++)
    {
      // cost computation:
      float cost_function = 0;
      for( float j = 0; j < seg_pointcloud[k].size(); j++)
      {
        cost_function += sqrt(pow(Xp.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
                              pow(Xp.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
                              pow(Xp.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );
      }

      //gradient computation:
      pcl::PointXYZ Gradient_Xp;
      for( float j = 0; j < seg_pointcloud[k].size(); j++)
      {
        Gradient_Xp.x += (Xp.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x) / 
                          sqrt(pow(Xp.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
                               pow(Xp.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
                               pow(Xp.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );

        Gradient_Xp.y += (Xp.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y) / 
                          sqrt(pow(Xp.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
                               pow(Xp.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
                               pow(Xp.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );

        Gradient_Xp.z += (Xp.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z) / 
                          sqrt(pow(Xp.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
                               pow(Xp.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
                               pow(Xp.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );
      }

      // xyz step in one step
      Xpi.x = Xp.x - theta * Gradient_Xp.x;
      Xpi.y = Xp.y - theta * Gradient_Xp.y;
      Xpi.z = Xp.z - theta * Gradient_Xp.z;

      //compute updated cost function
      float cost_function_update = 0;
      for( float j = 0; j < seg_pointcloud[k].size(); j++)
      {
        cost_function_update += sqrt(pow(Xpi.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
                                     pow(Xpi.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
                                     pow(Xpi.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );
      }

      //判断loss是否小于阈值
      loss = cost_function - cost_function_update;
      // cout << "loss: " << loss << endl;

      //loss 大于 最小误差， 继续迭代
      if(loss > epsilon)
      {
        Xp.x = Xpi.x;
        Xp.y = Xpi.y;
        Xp.z = Xpi.z;

        cost_function = cost_function_update;
      }

      //减少步长
      else if(cost_function_update - cost_function > epsilon)
      {
        theta = theta * 0.8;
      }

      //满足迭代条件，得到目标点
      else
      {
        Xp_goal.x = Xp.x;
        Xp_goal.y = Xp.y;
        Xp_goal.z = Xp.z;
        Xp_goal.b = 0;
        Xp_goal.g = 0;
        Xp_goal.r = 0;
        cloud_seamRegion->points.push_back( Xp_goal ) ; 

        break;
      }
    }

    // cout << "seg_count: " << k+1 << endl;
    // cout << "Xp_goal: " << Xp_goal << endl << endl;
    path_cloud->points.push_back( Xp_goal );
    path_cloud_showRviz->points.push_back( Xp_goal );

    // 插值， 构成完整轨迹, 每个path点之间插5个点
    if(k > 0)
    {
      pcl::PointXYZRGB p_add1, p_add2, p_add3, p_add4, p_add5, p_add6, p_add7;

      p_add1.x = (path_cloud->points[k - 1].x + path_cloud->points[k].x) / 2.0;
      p_add1.y = (path_cloud->points[k - 1].y + path_cloud->points[k].y) / 2.0;
      p_add1.z = (path_cloud->points[k - 1].z + path_cloud->points[k].z) / 2.0;
      cloud_seamRegion->points.push_back( p_add1 );
      path_cloud_showRviz->points.push_back( p_add1 );

      p_add2.x = (path_cloud->points[k - 1].x + p_add1.x) / 2.0;
      p_add2.y = (path_cloud->points[k - 1].y + p_add1.y) / 2.0;
      p_add2.z = (path_cloud->points[k - 1].z + p_add1.z) / 2.0;
      cloud_seamRegion->points.push_back( p_add2 );
      path_cloud_showRviz->points.push_back( p_add2 );

      p_add3.x = (p_add1.x + path_cloud->points[k].x) / 2.0;
      p_add3.y = (p_add1.y + path_cloud->points[k].y) / 2.0;
      p_add3.z = (p_add1.z + path_cloud->points[k].z) / 2.0;
      cloud_seamRegion->points.push_back( p_add3 );
      path_cloud_showRviz->points.push_back( p_add3 );

      p_add4.x = (path_cloud->points[k - 1].x + p_add2.x) / 2.0;
      p_add4.y = (path_cloud->points[k - 1].y + p_add2.y) / 2.0;
      p_add4.z = (path_cloud->points[k - 1].z + p_add2.z) / 2.0;
      cloud_seamRegion->points.push_back( p_add4 );
      path_cloud_showRviz->points.push_back( p_add4 );

      p_add5.x = (p_add1.x + p_add2.x) / 2.0;
      p_add5.y = (p_add1.y + p_add2.y) / 2.0;
      p_add5.z = (p_add1.z + p_add2.z) / 2.0;
      cloud_seamRegion->points.push_back( p_add5 );
      path_cloud_showRviz->points.push_back( p_add5 );

      p_add6.x = (p_add1.x + p_add3.x) / 2.0;
      p_add6.y = (p_add1.y + p_add3.y) / 2.0;
      p_add6.z = (p_add1.z + p_add3.z) / 2.0;
      cloud_seamRegion->points.push_back( p_add6 );
      path_cloud_showRviz->points.push_back( p_add6 );

      p_add7.x = (path_cloud->points[k].x + p_add3.x) / 2.0;
      p_add7.y = (path_cloud->points[k].y + p_add3.y) / 2.0;
      p_add7.z = (path_cloud->points[k].z + p_add3.z) / 2.0;
      cloud_seamRegion->points.push_back( p_add7 );
      path_cloud_showRviz->points.push_back( p_add7 );
    }
  }
 
  cout << "path_cloud->points.size(): "          << path_cloud->points.size() << endl;
  cout << "path_cloud_showRviz->points.size(): " << path_cloud_showRviz->points.size() << endl << endl;


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  vector<float> orientation_pathpoints = cylinder_pathComputation(cloud_seamRegion, path_cloud);

  return orientation_pathpoints;
}

Point3f circle_estimation(vector<Point3f> path_points);

vector<float> cylinder_pathComputation(PointCloud::Ptr cloud_seamRegion, PointCloud::Ptr path_cloud)
{
  vector<Point3f> path_points;
  for( float i = 0; i < path_cloud->points.size(); i++)
  {
    Point3f p;

    p.x = path_cloud->points[ i ].x;
    p.y = path_cloud->points[ i ].y;
    p.z = path_cloud->points[ i ].z;

    path_points.push_back( p );
  }
  
  path_cloud->clear();
  cout << "path_points.size(): "          << path_points.size() << endl;

  Point3f p_circle = circle_estimation(path_points);

  pcl::PointXYZRGB p ;
  p.x = p_circle.x;
  p.y = p_circle.y;
  p.z = p_circle.z;
  p.b = 0;
  p.g = 0;
  p.r = 200;
  cloud_seamRegion->points.push_back(p);


  vector<Point3f> cylinder_path_points;        
  Point3f path_cylinder;

  vector<float> orientation_pathpoints;
  float   k_orientation;

  for(int i = 0; i < path_points.size(); i++)
  {
    k_orientation = (p_circle.y - path_points[i].y) / (p_circle.x - path_points[i].x);
    cout << "yaw: " << atan( -1 / k_orientation ) * 180 / M_PI << endl;
    orientation_pathpoints.push_back( atan( -1 / k_orientation ) * 180 / M_PI );

    //////////////////////////////////////////////////////

    path_cylinder.x = path_points[i].x;//(2 * path_points[i].x) - p_circle.x;
    path_cylinder.y = path_points[i].y;//(2 * path_points[i].y) - p_circle.y;
    path_cylinder.z = path_points[i].z;//(2 * path_points[i].z) - p_circle.z;
    cout << "path_cylinder: " << path_cylinder << endl;

    cylinder_path_points.push_back( path_cylinder );

    // pcl::PointXYZRGB p ;
    // p.x = path_cylinder.x;
    // p.y = path_cylinder.y;
    // p.z = path_cylinder.z;
    // p.b = 0;
    // p.g = 0;
    // p.r = 200;
    // cloud_seamRegion->points.push_back(p);
  }
  cout << "orientation_pathpoints.size(): " << orientation_pathpoints.size() << endl;
  cout << "cylinder_path_points.size(): "      << cylinder_path_points.size() << endl;

  for(int i = 0; i < cylinder_path_points.size(); i++)
  {
    pcl::PointXYZRGB p;

    p.x = cylinder_path_points[i].x;
    p.y = cylinder_path_points[i].y;
    p.z = cylinder_path_points[i].z;

    path_cloud->points.push_back(p);
  }

  cout << "path_cloud->points.size(): "        << path_cloud->points.size() << endl;

  return  orientation_pathpoints;
}

Point3f circle_estimation(vector<Point3f> path_points)
{
  Point3f p_start, p_mid, p_end;

  //赋值给三个点
  p_start.x = path_points[0].x;                      p_start.y = path_points[0].y;                      p_start.z = path_points[0].z;
    p_mid.x = path_points[path_points.size() / 2].x;   p_mid.y = path_points[path_points.size() / 2].y;   p_mid.z = path_points[path_points.size() / 2].z;
    p_end.x = path_points[path_points.size() - 1].x;   p_end.y = path_points[path_points.size() - 1].y;   p_end.z = path_points[path_points.size() - 1].z;

  //算两条线的斜率
  float line1_k = (p_mid.y - p_start.y) / (p_mid.x - p_start.x);
  float line2_k = (p_mid.y -   p_end.y) / (p_mid.x -   p_end.x);

  //得到两条线直线方程
  cout << "line1_k: " << line1_k << endl;
  cout << "line2_k: " << line2_k << endl;
  // cout << "得到两条直线方程为：" << endl;
  cout << "y = " << line1_k << " * (x - p_mid.x) + p_mid.y: " << endl;
  cout << "y = " << line2_k << " * (x - p_mid.x) + p_mid.y: " << endl;

  //求两对点的中间点
  Point3f p_midpoint1, p_midpoint2;

  p_midpoint1.x = (p_start.x + p_mid.x) / 2; p_midpoint1.y = (p_start.y + p_mid.y) / 2; p_midpoint1.z = (p_start.z + p_mid.z) / 2;
  p_midpoint2.x = (  p_end.x + p_mid.x) / 2; p_midpoint2.y = (  p_end.y + p_mid.y) / 2; p_midpoint2.z = (  p_end.z + p_mid.z) / 2;
  // cout << "得到两中点为：" << endl;
  cout << "p_midpoint1：" << p_midpoint1 << endl;
  cout << "p_midpoint2：" << p_midpoint2 << endl;
  // cout << "相垂直的两条线斜率为：" << endl;
  cout << "-1 / line1_k:" << -1 / line1_k << endl;
  cout << "-1 / line2_k:" << -1 / line2_k << endl;
  // cout << "得到两条垂直直线方程为：" << endl;
  cout << "y = " <<  -1 / line1_k  << " * (x - " << p_midpoint1.x << " ) + " << p_midpoint1.y << endl;
  cout << "y = " <<  -1 / line2_k  << " * (x - " << p_midpoint2.x << " ) + " << p_midpoint2.y << endl;

  //求估计的圆心：
  Point3f p_circle;

  p_circle.x = ( ((-1 / line1_k) * p_midpoint1.x - p_midpoint1.y) - ((-1 / line2_k) * p_midpoint2.x - p_midpoint2.y) ) / ( (-1 / line1_k) - (-1 / line2_k) );
  p_circle.y = (-1 / line1_k) * (p_circle.x - p_midpoint1.x) + p_midpoint1.y; 

  for(int i = 0; i < path_points.size(); i++)
  {
    p_circle.z += path_points[i].z;
  }
  p_circle.z = p_circle.z / path_points.size();

  cout << "最后估计的圆中心为：" << endl;
  cout << "p_circle: " << p_circle << endl << endl;

  return p_circle;
}


 