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


Normal allPoint_normal_computation(Cloud::Ptr cloud_ptr)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  Normal::Ptr cloud_normals_ptr (new Normal);
  Normal& cloud_normals = *cloud_normals_ptr;

  ne.setRadiusSearch (0.005);
  ne.compute (cloud_normals);
  cout << "cloud_normals " << cloud_normals.size() << endl << endl;

  return cloud_normals;
}


void basic_normal_computation(Cloud::Ptr cloud_ptr, Normal cloud_normals, float *basic_normal_x, float *basic_normal_y, float *basic_normal_z)
{
  float total_pointcount = 0;

  for(float i = 0; i < cloud_ptr->points.size(); i++)
  { 
    if ( __isnan(cloud_normals[i].curvature) == true)
    {
      continue;
    }
    total_pointcount++;

    *basic_normal_x += cloud_normals[i].normal_x;
    *basic_normal_y += cloud_normals[i].normal_y;
    *basic_normal_z += cloud_normals[i].normal_z;
  }

  *basic_normal_x = 1.0 * *basic_normal_x / total_pointcount;
  *basic_normal_y = 1.0 * *basic_normal_y / total_pointcount;
  *basic_normal_z = 1.0 * *basic_normal_z / total_pointcount;

  cout << "initial points.size(): " << cloud_ptr->points.size() << endl;
  cout << "nan-point count: " << cloud_normals.size() - total_pointcount << endl;//858
  cout << "basic_normal_x: " << *basic_normal_x << endl;
  cout << "basic_normal_y: " << *basic_normal_y << endl;
  cout << "basic_normal_z: " << *basic_normal_z << endl << endl;

}



vector<float> Point_descriptor_computation(PointCloud::Ptr descriptor_cloud, Cloud::Ptr cloud_ptr, Normal cloud_normals, float basic_normal_x, float basic_normal_y, float basic_normal_z)
{
  vector<float> Dir_descriptor ;

  for(float i = 0; i < cloud_ptr->points.size(); i++)
  { 
    if ( __isnan(cloud_normals[i].curvature) == true)
    {
      continue;
    }

    float dir_descriptor;
    dir_descriptor = sqrt(pow(cloud_normals[i].normal_x - basic_normal_x, 2) + 
                          pow(cloud_normals[i].normal_y - basic_normal_y, 2) + 
                          pow(cloud_normals[i].normal_z - basic_normal_z, 2));

    Dir_descriptor.push_back( dir_descriptor );     

    pcl::PointXYZRGB p;
    p.x = cloud_ptr->points[i].x; 
    p.y = cloud_ptr->points[i].y;
    p.z = cloud_ptr->points[i].z;
    p.b = 0; 
    p.g = 0;
    p.r = 0;

    descriptor_cloud->points.push_back( p );        
  }

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



vector<float> Point_variance_computation(Cloud::Ptr cloud_tree_variance, PointCloud::Ptr descriptor_cloud, vector<float> Dir_descriptor, float *Var_descriptor_min, float *Var_descriptor_max)
{
  for(float i = 0; i < descriptor_cloud->points.size(); i++)
  { 
    pcl::PointXYZ p;

    p.x = descriptor_cloud->points[i].x;
    p.y = descriptor_cloud->points[i].y;
    p.z = descriptor_cloud->points[i].z;

    cloud_tree_variance->points.push_back( p );
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  // 创建一个 KdTree 对象
  kdtree.setInputCloud (cloud_tree_variance);  // 将前面创建的随机点云作为 KdTree 输入
  vector<int> pointIdxRadiusSearch; // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  vector<float> pointRadiusSquaredDistance;
  float radius = 0.005;

  vector<float> variance_descriptor ;

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

  for(float i = 0; i < variance_descriptor.size(); i++)
  { 
    if(i == 0)
    {
      *Var_descriptor_max = variance_descriptor[0];
      *Var_descriptor_min = variance_descriptor[0];
    }

    if (*Var_descriptor_max < variance_descriptor[i])
    {
      *Var_descriptor_max = variance_descriptor[i];
    }

    if(*Var_descriptor_min > variance_descriptor[i])
    {
      *Var_descriptor_min = variance_descriptor[i];
    }
  }
  cout << "Var_descriptor_max: "    << *Var_descriptor_max << endl;
  cout << "Var_descriptor_min: "    << *Var_descriptor_min << endl;

  cout << "cloud_tree_variance->points.size(): " << cloud_tree_variance->points.size() << endl;
  cout << "variance_descriptor.size(): " << variance_descriptor.size() << endl << endl;

  return variance_descriptor;
}




void exact_Target_regionPointcloud(PointCloud::Ptr cloud_tree_rm_irrelativePoint, Cloud::Ptr cloud_tree_variance, PointCloud::Ptr cloud_tree_variance_show)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_sreenout;  // 将前面创建的随机点云作为 KdTree 输入
  kdtree_sreenout.setInputCloud (cloud_tree_variance);
  vector<int> pointIdxRadiusSearch_sreenout;         // 创建两个向量，分别存放近邻的索引值、近邻的中心距
  vector<float> pointRadiusSquaredDistance_sreenout;
  float radius_sreenout = 0.005;  // 指定随机半径

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
  Cloud::Ptr ec_tree_cloud (new Cloud);
  for(float i = 0; i < cloud_tree_rm_irrelativePoint->points.size(); i++)
  { 
    pcl::PointXYZ p;

    p.x = cloud_tree_rm_irrelativePoint->points[ i ].x;
    p.y = cloud_tree_rm_irrelativePoint->points[ i ].y;
    p.z = cloud_tree_rm_irrelativePoint->points[ i ].z;

    ec_tree_cloud->points.push_back( p );
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr ec_tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ec_tree->setInputCloud (ec_tree_cloud);//创建点云索引向量，用于存储实际的点云信息

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> EC;

  EC.setClusterTolerance (0.01); //设置近邻搜索的搜索半径为2cm
  EC.setMinClusterSize (1);//设置一个聚类需要的最少点数目为100
  EC.setMaxClusterSize (10000000); //设置一个聚类需要的最大点数目为25000
  EC.setSearchMethod (ec_tree);//设置点云的搜索机制
  EC.setInputCloud (ec_tree_cloud);
  EC.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中
  
  cout << "ec_tree_cloud->points.size(): "  << ec_tree_cloud->points.size() << endl ;
  cout << "cluster_indices.size(): " << cluster_indices.size() << endl;

  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointXYZRGB p;

    Cloud::Ptr cloud_cluster (new Cloud);
    
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (ec_tree_cloud->points[*pit]);  
    }

    // cout << "cloud_cluster->points.size(): " << cloud_cluster->points.size() << endl;

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



vector< vector<int> > Segment_seam_region(PointCloud::Ptr cloud_seamRegion)
{
  float min_x = 0, max_x = 0;
  float min_y = 0, max_y = 0;
  float min_z = 0, max_z = 0;
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

  float segmentation_count = 50, seg_interval = range_x / segmentation_count;
  vector< vector<int> > seg_pointcloud;
  seg_pointcloud.resize(segmentation_count);

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

  float seg_counttt = 0;
  for( float j = 0; j < seg_pointcloud.size(); j++)
  {
    cout << "seg_pointcloud[j].size(): " << seg_pointcloud[j].size() << endl; 
    seg_counttt += seg_pointcloud[j].size();
  }

  cout << "seg_counttt:" << seg_counttt << endl;
  cout << "cloud_seamRegion->points.size(): " << cloud_seamRegion->points.size() << endl << endl;

  //show segmentation

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
  cout <<"segmentation show!!!" << endl << endl;

  return seg_pointcloud;
}




void Path_Generation(vector< vector<int> > seg_pointcloud, PointCloud::Ptr cloud_seamRegion, PointCloud::Ptr path_cloud, PointCloud::Ptr path_cloud_showRviz)
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

  for(int k = 0; k < seg_pointcloud.size(); k++)
  {
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

      Xpi.x = Xp.x - theta * Gradient_Xp.x;
      Xpi.y = Xp.y - theta * Gradient_Xp.y;
      Xpi.z = Xp.z - theta * Gradient_Xp.z;

      float cost_function_update = 0;
      for( float j = 0; j < seg_pointcloud[k].size(); j++)
      {
        cost_function_update += sqrt(pow(Xpi.x - cloud_seamRegion->points[ seg_pointcloud[k][j] ].x, 2) + 
                                     pow(Xpi.y - cloud_seamRegion->points[ seg_pointcloud[k][j] ].y, 2) + 
                                     pow(Xpi.z - cloud_seamRegion->points[ seg_pointcloud[k][j] ].z, 2) );
      }

      //判断loss是否小于阈值
      loss = cost_function - cost_function_update;
      // cout << "loss: " << loss << endl << endl;

      if(loss > epsilon)
      {
        Xp.x = Xpi.x;
        Xp.y = Xpi.y;
        Xp.z = Xpi.z;

        cost_function = cost_function_update;
      }

      else if(cost_function_update - cost_function > epsilon)
      {
        theta = theta * 0.8;
      }

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
    cout << "seg_count: " << k+1 << endl;
    cout << "Xp_goal: " << Xp_goal << endl << endl;
    path_cloud->points.push_back( Xp_goal );
    path_cloud_showRviz->points.push_back( Xp_goal );

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
 
  cout << "path_cloud->points.size(): " << path_cloud->points.size() << endl;
  cout << "path_cloud_showRviz->points.size(): " << path_cloud_showRviz->points.size() << endl << endl;

}