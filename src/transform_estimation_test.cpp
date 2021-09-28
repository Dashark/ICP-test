#include "transform_xyz_ptz.h"

#include <iostream>

int main (int argc, char** argv)
{
  printfEST();  
  float mat[16] = {
    -0.969271f, -0.199635f,  0.143738f,   157.107f,
    0.0451132f,  -0.71865f, -0.693907f,   1713.21f,
     0.241825f, -0.666099f,  0.705572f,   32.4196f,
        0.0f,         0.0f,         0.0f,         1.0f
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
  xyz[0] = 5.5f;
  xyz[1] = 1.0f;
  xyz[2] = 1.0f;
  xyz[0] = 5.5f;
  xyz[1] = -1.0f;
  xyz[2] = 1.0f;
  std::cout << "PTZ count: " << transformPTZ(xyz, ptz, 15) << std::endl;  // 坐标转换
  for (int i = 0; i < 15; ++i) {  // 输出转换的结果
    std::cout << ptz[i] << " ";
    if ((i+1) % 3 == 0)
      std::cout << std::endl;
  }
  return ( 0 );
}

