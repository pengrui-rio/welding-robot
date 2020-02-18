#include <algorithm.h>



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

 