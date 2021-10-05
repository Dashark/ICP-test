#ifndef _TRANSFORM_XYZ_PTZ_
#define _TRANSFORM_XYZ_PTZ_

/**
 * @brief 从Java端传送过来的4 x 4 转换矩阵
 * 
 * @param matrix 按照一维数组存储的矩阵，行优先
 * @param size 数组大小，缺省16
 */
void tranformMatrix(float matrix[], int size = 16);
/**
 * @brief 将XYZ值转换为PTZ值
 * 
 * @param xyz 从Java端传送的雷达坐标值
 * @param ptz 用于控制摄像头云台的PTZ值
 * @param size xyz数组的大小
 * @return int 返回PTZ数组的大小
 */
int transformPTZ(float xyz[], float ptz[], int size);

/**
 * @brief 将XYZ值转换为PTZ值
 * 
 * @param xyz 从雷达中同一目标的坐标值
 * @param size xyz数组的大小
 * @param p 球机的P值
 * @param t 球机的T值
 * @param z 球机的Z值
 * @return int 固定为1
 */
int transformPTZ(float xyz[], int size, float &p, float &t, float &z);
void printfEST();

#endif