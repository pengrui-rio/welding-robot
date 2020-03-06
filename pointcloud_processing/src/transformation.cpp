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
  T_E_C <<   1,   2.7798e-18,            0,   -0.0401405,
  -5.57321e-19,            1,  2.71051e-20,    -0.116625,
    1.0842e-19,            0,            1,    0.0707586,
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


pcl::PointXYZ realsense_position_acquisition(geometry_msgs::Pose Base_End)
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
  T_E_C <<   1,   2.7798e-18,            0,   -0.0401405,
  -5.57321e-19,            1,  2.71051e-20,    -0.116625,
    1.0842e-19,            0,            1,    0.0707586,
             0,            0,            0,            1;
  // cout << "T_E_C: " << endl << T_E_C << endl;


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
