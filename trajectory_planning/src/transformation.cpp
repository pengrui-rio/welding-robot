// #include <iostream>
#include <transformation.h>
// PCL lib
 

//右手坐标系
void rotate_z(float x, float y, float z, float angle, float* x_output, float* y_output, float* z_output) 
{
    float atopi = angle / 180.0 * M_PI;
    *x_output = x * cos(atopi) - y * sin(atopi);
    *y_output = y * cos(atopi) + x * sin(atopi);
    *z_output = z;
 }

void rotate_x(float x, float y, float z, float angle, float* x_output, float* y_output, float* z_output)
{
    float atopi = angle / 180.0 * M_PI;
    *z_output = z * cos(atopi) + y * sin(atopi);
    *y_output = y * cos(atopi) - z * sin(atopi);
    *x_output = x;
 }

void rotate_y(float x, float y, float z, float angle, float* x_output, float* y_output, float* z_output)
{
    float atopi = angle / 180.0 * M_PI;
    *x_output = x * cos(atopi) - z * sin(atopi);
    *z_output = z * cos(atopi) + x * sin(atopi);
    *y_output = y;
 }



//四元数 -> 旋转矩阵
void Quaternion_to_RotationMatrix(float x, float y, float z, float w, float R[9])
{
  float r11 = 0, r12 = 0, r13 = 0;
  float r21 = 0, r22 = 0, r23 = 0; 
  float r31 = 0, r32 = 0, r33 = 0;

  R[0]  = 1 - 2 * y * y - 2 * z * z;
  R[1]  =     2 * x * y - 2 * z * w;
  R[2]  =     2 * x * z + 2 * y * w;

  R[3]  =     2 * x * y + 2 * z * w;
  R[4]  = 1 - 2 * x * x - 2 * z * z;
  R[5]  =     2 * y * z - 2 * x * w;

  R[6]  =     2 * x * z - 2 * y * w;
  R[7]  =     2 * y * z + 2 * x * w;
  R[8]  = 1 - 2 * x * x - 2 * y * y;

}

//欧拉角 -> 四元数
void euler_to_quaternion(float Yaw, float Pitch, float Roll, float Q[4])
{
  float yaw   = Yaw   * M_PI / 180 ;
  float pitch = Roll  * M_PI / 180 ;
  float roll  = Pitch * M_PI / 180 ;

  float qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
  float qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
  float qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
  float qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
 
  Q[0] = qx;
  Q[1] = qy;
  Q[2] = qz;
  Q[3] = qw;
}


//欧拉角 -> 旋转矩阵
void euler_to_RotationMatrix(float Yaw, float Pitch, float Roll, float R[9])
{
  float yaw   = Yaw   * M_PI / 180 ;
  float pitch = Roll  * M_PI / 180 ;
  float roll  = Pitch * M_PI / 180 ;

  float c1 = cos(yaw);
  float c2 = cos(pitch);
  float c3 = cos(roll);
  float s1 = sin(yaw);
  float s2 = sin(pitch);
  float s3 = sin(roll);

  R[0]  = c1 * c3 + s1 * s2 * s3;
  R[1]  = c3 * s1 * s2 - c1 * s3;
  R[2]  = c2 * s1;

  R[3]  = c2 * s3;
  R[4]  = c2 * c3;
  R[5]  = -s2;

  R[6]  = c1 * s2 * s3 - s1 * c3;
  R[7]  = s1 * s3 + c1 * c3 * s2;
  R[8]  = c1 * c2;
}

 
pcl::PointXYZ camera_to_base_transform(geometry_msgs::Pose Base_End, pcl::PointXYZ Cam_Object)
{

  //Base -> End:
  float T_B_E_Q[4];
  T_B_E_Q[0] = Base_End.orientation.x;
  T_B_E_Q[1] = Base_End.orientation.y;
  T_B_E_Q[2] = Base_End.orientation.z;
  T_B_E_Q[3] = Base_End.orientation.w;

  float T_B_E_r[9];
  Quaternion_to_RotationMatrix(T_B_E_Q[0], T_B_E_Q[1], T_B_E_Q[2], T_B_E_Q[3], T_B_E_r);   
  // euler_to_RotationMatrix(0, -180, 0, T_B_E_r);
  Matrix4d T_B_E; // Base -> End
  T_B_E << T_B_E_r[0], T_B_E_r[1], T_B_E_r[2], Base_End.position.x,
           T_B_E_r[3], T_B_E_r[4], T_B_E_r[5], Base_End.position.y,
           T_B_E_r[6], T_B_E_r[7], T_B_E_r[8], Base_End.position.z,
           0,          0,          0,          1;
  // cout << "T_B_E: " << endl << T_B_E << endl;


  // End -> Camera
  Matrix4d T_E_C;
  T_E_C <<   1,   2.7798e-18,            0,   -0.0355791,
  -5.57321e-19,            1,  2.71051e-20,    -0.116142,
    1.0842e-19,            0,            1,    0.0708987,
             0,            0,            0,            1;
  // cout << "T_E_C: " << endl << T_E_C << endl;


  // Base -> Camera
  Matrix4d T_B_C;
  T_B_C = T_B_E * T_E_C;
  // cout << "T_B_C: " << endl << T_B_C << endl;


  // Camera -> object  0.0691944 -0.0164191
  Matrix4d T_C_o; 
  float T_C_o_Q[4];
  euler_to_quaternion(0, 0, 0, T_C_o_Q);
  float T_C_o_r[9];
  Quaternion_to_RotationMatrix(T_C_o_Q[0], T_C_o_Q[1], T_C_o_Q[2], T_C_o_Q[3], T_C_o_r);//camera_color_optical_frame
  // euler_to_RotationMatrix(-90, -180, 0, T_C_o_r);
  T_C_o << T_C_o_r[0], T_C_o_r[1], T_C_o_r[2], Cam_Object.x,
           T_C_o_r[3], T_C_o_r[4], T_C_o_r[5], Cam_Object.y,
           T_C_o_r[6], T_C_o_r[7], T_C_o_r[8], Cam_Object.z,
           0,          0,          0,          1;
  // cout << "T_C_o: " << endl << T_C_o << endl;

  // result by calibration:
  Matrix4d T_R;
  T_R = T_B_C * T_C_o;
  // cout << "T_R: " << endl << T_R << endl;

  pcl::PointXYZ object_position_world;
  object_position_world.x = T_R(0,3);
  object_position_world.y = T_R(1,3);
  object_position_world.z = T_R(2,3);

  return  object_position_world;
}


pcl::PointXYZ realsense_position_acquisition(tf::StampedTransform transform)
{
  geometry_msgs::Pose Base_End;
  Base_End.position.x = transform.getOrigin().x();    Base_End.position.y = transform.getOrigin().y();    Base_End.position.z = transform.getOrigin().z();
  Base_End.orientation.x = transform.getRotation().x(); Base_End.orientation.y = transform.getRotation().y(); Base_End.orientation.z = transform.getRotation().z(); Base_End.orientation.w = transform.getRotation().w();
 ///////////////////////////////////////////////////

  //Base -> End:
  float T_B_E_Q[4];
  T_B_E_Q[0] = Base_End.orientation.x;
  T_B_E_Q[1] = Base_End.orientation.y;
  T_B_E_Q[2] = Base_End.orientation.z;
  T_B_E_Q[3] = Base_End.orientation.w;

  float T_B_E_r[9];
  Quaternion_to_RotationMatrix(T_B_E_Q[0], T_B_E_Q[1], T_B_E_Q[2], T_B_E_Q[3], T_B_E_r);   
  // euler_to_RotationMatrix(0, -180, 0, T_B_E_r);
  Matrix4d T_B_E; // Base -> End
  T_B_E << T_B_E_r[0], T_B_E_r[1], T_B_E_r[2], Base_End.position.x,
           T_B_E_r[3], T_B_E_r[4], T_B_E_r[5], Base_End.position.y,
           T_B_E_r[6], T_B_E_r[7], T_B_E_r[8], Base_End.position.z,
           0,          0,          0,          1;
  // cout << "T_B_E: " << endl << T_B_E << endl;


  // End -> Camera
  Matrix4d T_E_C;
  T_E_C <<   1,   2.7798e-18,            0,   -0.0355791,
  -5.57321e-19,            1,  2.71051e-20,    -0.116142,
    1.0842e-19,            0,            1,    0.0738987,
             0,            0,            0,            1;


  // Base -> Camera
  Matrix4d T_B_C;
  T_B_C = T_B_E * T_E_C;
  // cout << "T_B_C: " << endl << T_B_C << endl;

  pcl::PointXYZ realsense_position;
  realsense_position.x = T_B_C(0,3);
  realsense_position.y = T_B_C(1,3);
  realsense_position.z = T_B_C(2,3);

  return realsense_position;
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void analyze_realsense_data(PointCloud::Ptr cloud)
{
  // 遍历深度图
  for (int m = 0; m < depth_pic.rows; m++)  //480
  {
    for (int n = 0; n < depth_pic.cols; n++) //640
    {
      // 获取深度图中(m,n)处的值
      float d = depth_pic.ptr<float>(m)[n]; 
      // d 可能没有值，若如此，跳过此点
      if (d == 0)
          continue;

      // d 存在值，则向点云增加一个点
      pcl::PointXYZRGB p, p_z, p_x, p_y;
      pcl::PointXYZRGB p_transform;

      // 计算这个点的空间坐标
      p.z = double(d) / camera_factor  ;
      p.x = (n - camera_cx) * p.z / camera_fx; 
      p.y = (m - camera_cy) * p.z / camera_fy;

      //筛选位于机械臂工作空间内的点
      if( (p.x < -1 || p.x > 1) || (p.y < -1 || p.y > 1) || (p.z < -1 || p.z > 1) )
      {
        continue;
      }

      // 从rgb图像中获取它的颜色
      // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
      p.b = color_pic.ptr<uchar>(m)[n*3];
      p.g = color_pic.ptr<uchar>(m)[n*3+1];
      p.r = color_pic.ptr<uchar>(m)[n*3+2];


      cloud->points.push_back( p );        
    }
  }
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void coordinate_transformation(tf::StampedTransform transform, PointCloud::Ptr camera_pointcloud, PointCloud::Ptr map_pointcloud, Cloud::Ptr cloud_ptr)
{
  geometry_msgs::Pose Base_End;
  Base_End.position.x = transform.getOrigin().x();    Base_End.position.y = transform.getOrigin().y();    Base_End.position.z = transform.getOrigin().z();
  Base_End.orientation.x = transform.getRotation().x(); Base_End.orientation.y = transform.getRotation().y(); Base_End.orientation.z = transform.getRotation().z(); Base_End.orientation.w = transform.getRotation().w();
  ///////////////////////////////////////////////////////////////////////////////////////

  for (float i = 0; i < camera_pointcloud->points.size(); i++)  //480
  {
    pcl::PointXYZRGB p_pushback; 
    pcl::PointXYZ p_cloud_ptr;
    pcl::PointXYZ Cam_Object;

    Cam_Object.x    = camera_pointcloud->points[i].x; 
    Cam_Object.y    = camera_pointcloud->points[i].y; 
    Cam_Object.z    = camera_pointcloud->points[i].z; 

    pcl::PointXYZ p_tranform = camera_to_base_transform(Base_End, Cam_Object);
    
    p_cloud_ptr.x = p_pushback.x = p_tranform.x; 
    p_cloud_ptr.y = p_pushback.y = p_tranform.y;  
    p_cloud_ptr.z = p_pushback.z = p_tranform.z; 

    p_pushback.b = camera_pointcloud->points[i].b;
    p_pushback.g = camera_pointcloud->points[i].g;
    p_pushback.r = camera_pointcloud->points[i].r;

    map_pointcloud->points.push_back( p_pushback );    

    cloud_ptr->points.push_back( p_cloud_ptr );
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Cloud::Ptr cloud_ptr_origin_copy(Cloud::Ptr cloud_ptr_new)
{
  Cloud::Ptr cloud_ptr_origin (new Cloud);
  for(float i = 0; i < cloud_ptr_new->points.size(); i++)
  {
    pcl::PointXYZ p;
    p.x = cloud_ptr_new->points[i].x;
    p.y = cloud_ptr_new->points[i].y;
    p.z = cloud_ptr_new->points[i].z;
    cloud_ptr_origin->points.push_back( p );    
  }

  return cloud_ptr_origin;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Cloud::Ptr input_pointcloud_filter(int process_count, int process_count_limit, Cloud::Ptr cloud_ptr, Cloud::Ptr cloud_ptr_filter)
{
  for(float i = 0; i < cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZ p;
    p.x = cloud_ptr->points[i].x;
    p.y = cloud_ptr->points[i].y;
    p.z = cloud_ptr->points[i].z;
    cloud_ptr_filter->points.push_back( p );    
  }

  ////start filter
  Cloud::Ptr Cloud_filtered (new Cloud);
  if(process_count >= process_count_limit)
  {
    float radius = 0.001;

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize( radius, radius, radius );
    voxel.setInputCloud( cloud_ptr_filter );
    voxel.filter( *Cloud_filtered );

    cout << "input pointcloud filtering done!!!" << endl;
  }
  return Cloud_filtered;
}

// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // Eigen::Quaterniond Transform_AngleAxisd_Quatenion(vector<Point3f> Normal_Vector)
// void Transform_AngleAxisd_Quatenion(Cloud::Ptr PathPoint_Position)
// {
//   vector<Point3f> move_Vector;
//   for(float i = 0; i < cloud_ptr->points.size() - 1; i++)
//   {
//     Point3f v;
//     if(cloud_ptr->points[i].x < cloud_ptr->points[i+1].x)
//     {
//       v.x = cloud_ptr->points[i+1].x - cloud_ptr->points[i].x;
//       v.y = cloud_ptr->points[i+1].y - cloud_ptr->points[i].y;
//     }
//     else
//     {
//       v.x = cloud_ptr->points[i].x - cloud_ptr->points[i+1].x;
//       v.y = cloud_ptr->points[i].y - cloud_ptr->points[i+1].y;
//     }
//     move_Vector.push_back(v);
//   }
//   cout << "move_Vector.size()" << move_Vector.size() << endl;

//   // for(int i = 0; i < Normal_Vector.size(); i++)
//   // {
//     // Eigen::AngleAxisd rotation_vector(-M_PI / 3, Eigen::Vector3d(0.020818098, -0.032153312, 0.99926615) );

//     // Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 1.0/sqrt(2), 1.0/sqrt(2)) );
//     // Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
//     // // cout << "quaternion = \n" << q.x() << " " << q.y() << " " << q.z() << " " << q.w() <<  endl;

//     // Eigen::Vector3d eulerAngle=rotation_vector.matrix().eulerAngles(2,1,0);


//   for(float i = 0; i < move_Vector.size() - 1; i++)
//   {
//     double x = move_Vector[i].x, y = move_Vector[i].y;//, z = -1.0/sqrt(2);
//     double pitch = 0, roll = 0, yaw = 0;

//     //yaw:
//     if(y > 0 && x > 0)
//       yaw = 0   + atan2(abs(y), abs(x)) * 180 / M_PI;
//     if(y > 0 && x < 0)
//       yaw = 90  + atan2(abs(x), abs(y)) * 180 / M_PI;
//     if(y < 0 && x < 0)
//       yaw = 180 + atan2(abs(y), abs(x)) * 180 / M_PI;
//     if(y < 0 && x > 0)
//       yaw = 270 + atan2(abs(x), abs(y)) * 180 / M_PI;

//     float q[4];
//     euler_to_quaternion(yaw, pitch, roll, q);

//     double px = 0.056;//double px = -0.04;
//     double py = 0.328;
//     double pz = 0.1;
//     float T_o_B_q[4];
//     double qx = q[0];
//     double qy = q[1];
//     double qz = q[2];
//     double qw = q[3];

//     tf::Quaternion rotation (qx,qy,qz,qw);
//     tf::Vector3 origin (px,py,pz);
//     tf::Transform t (rotation, origin);

//     std::string markerFrame = "GT_ar_marker_";
//     std::stringstream out;
//     out << 1;
//     std::string id_string = out.str();
//     markerFrame += id_string;
//     tf::StampedTransform GT_Marker (t, ros::Time::now(), "/base_link", markerFrame.c_str());
//     tf_broadcaster.sendTransform(GT_Marker);
//   }

//     // //pitch:
//     // if(y > 0 && z > 0)
//     //   pitch = 270 + atan2(abs(z), abs(y)) * 180 / M_PI;
//     // if(y > 0 && z < 0)
//     //   pitch = 180 + atan2(abs(y), abs(z)) * 180 / M_PI;
//     // if(y < 0 && z < 0)
//     //   pitch =  90 + atan2(abs(z), abs(y)) * 180 / M_PI;
//     // if(y < 0 && z > 0)
//     //   pitch =   0 + atan2(abs(y), abs(z)) * 180 / M_PI;

//     // //roll:
//     // if(y > 0 && z > 0)
//     //   pitch = 270 + atan2(abs(z), abs(y)) * 180 / M_PI;
//     // if(y > 0 && z < 0)
//     //   pitch = 180 + atan2(abs(y), abs(z)) * 180 / M_PI;
//     // if(y < 0 && z < 0)
//     //   pitch =  90 + atan2(abs(z), abs(y)) * 180 / M_PI;
//     // if(y < 0 && z > 0)
//     //   pitch =   0 + atan2(abs(y), abs(z)) * 180 / M_PI;

//     // //yaw:
//     // if(y > 0 && x > 0)
//     //   yaw = 0   + atan2(abs(y), abs(x)) * 180 / M_PI;
//     // if(y > 0 && x < 0)
//     //   yaw = 90  + atan2(abs(x), abs(y)) * 180 / M_PI;
//     // if(y < 0 && x < 0)
//     //   yaw = 180 + atan2(abs(y), abs(x)) * 180 / M_PI;
//     // if(y < 0 && x > 0)
//     //   yaw = 270 + atan2(abs(x), abs(y)) * 180 / M_PI;

//     // // cout << "eulerAngle: " << eulerAngle * 180 / M_PI << endl;
//     // cout << "eulerAngle: " <<  pitch << " " << roll << " " << yaw << endl;
//   // }

//     // double px = 0.056;//double px = -0.04;
//     // double py = 0.328;
//     // double pz = 0.1;
//     // float T_o_B_q[4];
//     // double qx = q.x();
//     // double qy = q.y();
//     // double qz = q.z();
//     // double qw = q.w();

//     // tf::Quaternion rotation (qx,qy,qz,qw);
//     // tf::Vector3 origin (px,py,pz);
//     // tf::Transform t (rotation, origin);

//     // std::string markerFrame = "GT_ar_marker_";
//     // std::stringstream out;
//     // out << 1;
//     // std::string id_string = out.str();
//     // markerFrame += id_string;
//     // tf::StampedTransform GT_Marker (t, ros::Time::now(), "/base_link", markerFrame.c_str());
//     // tf_broadcaster. sendTransform(GT_Marker);


//   return q;
// }



