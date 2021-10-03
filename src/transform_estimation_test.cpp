#include "transform_xyz_ptz.h"

#include <iostream>

int main (int argc, char** argv)
{
  printfEST();  
  float mat[16] = {
    0.0767312f,  -0.997038f, 0.00527448f,    1700.79f
    0.811204f,  0.0655036f,   0.581083f,    9.44269f
    -0.579708f, -0.0403085f,   0.813827f,    17.2098f
         0.0f,          0.0f,          0.0f,          1.0f
  };
  /*
  for (int i = 0; i < 16; ++i) {
    mat[i] = 1.0f;
  }*/
  tranformMatrix(mat);   // 初始化转换矩阵
  printfEST();  
  float xyz[16] = {0.1f}, ptz[16];
  for (int i = 0; i < 15; ++i) {  // 测试用的雷达坐标
    xyz[i] = 0.1f;
  }
  xyz[0] = 1.634f;
  xyz[1] = -3.11f;
  xyz[2] = 1.5f;
  xyz[3] = 1.946f;
  xyz[4] = 0.246f;
  xyz[5] = 1.5f;
  std::cout << "PTZ count: " << transformPTZ(xyz, ptz, 6) << std::endl;  // 坐标转换
  for (int i = 0; i < 15; ++i) {  // 输出转换的结果
    std::cout << ptz[i] << " ";
    if ((i+1) % 3 == 0)
      std::cout << std::endl;
  }
  return ( 0 );
}

