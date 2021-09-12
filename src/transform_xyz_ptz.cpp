#include "transform_xyz_ptz.h"

#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/common/centroid.h>
#include <pcl/console/parse.h>
#include <boost/random.hpp>

static Eigen::Matrix4f transformation_est; /* {
          {1, 1, 1, 1},
          {1, 1, 1, 1},
          {1, 1, 1, 1},
          {1, 1, 1, 1},
  };*/ // 坐标变换矩阵
void printfEST()
{
  std::cout << "estimated transformation " << std::endl << transformation_est.matrix()  << std::endl;
  
}
/**
 * @brief 从Java端传送过来的4 x 4 转换矩阵
 * 
 * @param matrix 按照一维数组存储的矩阵，行优先
 * @param size 数组大小，缺省16
 */
void tranformMatrix(float matrix[], int size) {
  for (int i = 0; i < size; ++i) {
    int row = i / 4;
    int col = i % 4;
    transformation_est(row, col) = matrix[i];
  }
}
/**
 * @brief 将XYZ值转换为PTZ值
 * 
 * @param xyz 从Java端传送的雷达坐标值
 * @param ptz 用于控制摄像头云台的PTZ值
 * @param size xyz数组的大小
 * @return int 返回PTZ数组的大小
 */
int transformPTZ(float xyz[], float ptz[], int size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
  for (int i = 0; i < size; i += 3) {
    cloud_source->push_back(pcl::PointXYZ (xyz[i], xyz[i+1], xyz[i+2]));
  }
  // create target point cloud
  pcl::transformPointCloud ( *cloud_source, *cloud_target, transformation_est);
  
  int i = 0;
  for (const auto& point : *cloud_target) {
    ptz[i] = point.x;
    ptz[i+1] = point.y;
    ptz[i+2] = point.z;
    i += 3;
  }
  return i;
}

