#include "transform_xyz_ptz.h"

#include <iostream>

int main (int argc, char** argv)
{
  printfEST();  
  float mat[16] = {1};
  for (int i = 0; i < 16; ++i) {
    mat[i] = 1.0f;
  }
  tranformMatrix(mat);   // 初始化转换矩阵
  printfEST();  
  float xyz[16] = {0.1f}, ptz[16];
  for (int i = 0; i < 15; ++i) {  // 测试用的雷达坐标
    xyz[i] = 0.1f;
  }
  std::cout << "PTZ count: " << transformPTZ(xyz, ptz, 15) << std::endl;  // 坐标转换
  for (int i = 0; i < 15; ++i) {  // 输出转换的结果
    std::cout << ptz[i] << " ";
    if ((i+1) % 3 == 0)
      std::cout << std::endl;
  }
  return ( 0 );
}

