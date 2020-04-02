// #include <iostream>
#include <reconstruction_transformation.h>
// PCL lib


 

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void coordinate_transformation(tf::StampedTransform transform, PointCloud::Ptr camera_pointcloud, PointCloud::Ptr cam_pc_transform, Cloud::Ptr cloud_ptr)
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

    cam_pc_transform->points.push_back( p_pushback );    

    cloud_ptr->points.push_back( p_cloud_ptr );
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointXYZ realsense_position_acquisition(tf::StampedTransform transform)
{
  geometry_msgs::Pose Base_End;
  Base_End.position.x = transform.getOrigin().x();    Base_End.position.y = transform.getOrigin().y();    Base_End.position.z = transform.getOrigin().z();
  Base_End.orientation.x = transform.getRotation().x(); Base_End.orientation.y = transform.getRotation().y(); Base_End.orientation.z = transform.getRotation().z(); Base_End.orientation.w = transform.getRotation().w();
  // cout << "Base_End: " << Base_End << endl;
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
geometry_msgs::Pose Torch_to_End_transform(tf::StampedTransform transform_tool02torch)
{
  Eigen::Quaterniond q(transform_tool02torch.getRotation().w(), 
                       transform_tool02torch.getRotation().x(), 
                       transform_tool02torch.getRotation().y(),
                       transform_tool02torch.getRotation().z());

  Matrix3d RotationMatrix = q.toRotationMatrix();

  Matrix4d T_tool02torch;
  T_tool02torch << RotationMatrix(0,0), RotationMatrix(0,1), RotationMatrix(0,2), transform_tool02torch.getOrigin().x(),
                   RotationMatrix(1,0), RotationMatrix(1,1), RotationMatrix(1,2), transform_tool02torch.getOrigin().y(),
                   RotationMatrix(2,0), RotationMatrix(2,1), RotationMatrix(2,2), transform_tool02torch.getOrigin().z(),
                   0                  , 0                  , 0                  , 1;
    
  // Eigen::Vector3d euler_angles = RotationMatrix.eulerAngles(2, 1, 0) * 180 / M_PI; 
  // cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << endl;

  // cout << "T_tool02torch =\n" << T_tool02torch << endl;      


  // Eigen::AngleAxisd rotation_vector(RotationMatrix);
  // cout << "rotation_vector "  << "angle is: " << rotation_vector.angle() * (180 / M_PI) 
  //                             << " axis is: " << rotation_vector.axis().transpose() << endl;

}

 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void model_3D_reconstruction(bool &receive_capture_command, 
                             int receive_capture_count,
                             string dataset_folder_path, 
                             pcl::PointXYZ realsense_position, 
                             PointCloud::Ptr cam_pc_transform_pointcloud, 
                             PointCloud::Ptr map_pointcloud)
{

  // cout << "realsense_position: " << realsense_position  << endl;
  
  if(receive_capture_command == true)
  {
    //receive_capture_count -> string
    ostringstream stream;
    stream << receive_capture_count;
    string ith_frame = stream.str();

    //define pointcloud_name
    string pointcloud_name;
    pointcloud_name = dataset_folder_path + "/" + ith_frame + "_frame.pcd";

    //save realsense_position
    string file_realsense_position;
    file_realsense_position = dataset_folder_path + "/" + ith_frame + "_frame.txt";
    ofstream out(file_realsense_position.c_str());
    out << realsense_position.x << "\t" << realsense_position.y << "\t" << realsense_position.z << endl; 
    out.close();

    //create single_transform_pointcloud
    PointCloud::Ptr single_transform_pointcloud  (new PointCloud);

    //add pointcloud
    for (float i = 0; i < cam_pc_transform_pointcloud->points.size(); i++)  
    {
      pcl::PointXYZRGB p; 

      p.x = cam_pc_transform_pointcloud->points[i].x; 
      p.y = cam_pc_transform_pointcloud->points[i].y; 
      p.z = cam_pc_transform_pointcloud->points[i].z; 

      p.b = cam_pc_transform_pointcloud->points[i].b;
      p.g = cam_pc_transform_pointcloud->points[i].g;
      p.r = cam_pc_transform_pointcloud->points[i].r;

      single_transform_pointcloud->points.push_back( p ); 
      map_pointcloud->points.push_back( p );    
    } 
    receive_capture_command = false;
    cout << "add one frame to model! " << endl;

    //record single_transform_pointcloud
    single_transform_pointcloud->width = 1;
    single_transform_pointcloud->height = single_transform_pointcloud->points.size();

    cout << "single_transform_pointcloud->points.size()" << single_transform_pointcloud->points.size() << endl;
    pcl::PCDWriter writer;
    writer.write(pointcloud_name, *single_transform_pointcloud, false) ;
    single_transform_pointcloud->points.clear();



    // map_pointcloud->width = 1;
    // map_pointcloud->height = map_pointcloud->points.size();

    // cout << "map_pointcloud->points.size()" << map_pointcloud->points.size() << endl;
    // pcl::PCDWriter writer;
    // writer.write("/home/rick/Documents/a_system/src/trajectory_planning/src/model_3D.pcd", *map_pointcloud, false) ;

  }
}
