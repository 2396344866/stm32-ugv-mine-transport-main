#ifndef UWB_2D_H
#define UWB_2D_H

#include "stm32f10x.h"

typedef struct { float x; float y; } Point2D;

/* 二维定位（三基站一标签，TWR 测距 + 最小二乘三边定位）
 * anchors: 3 个基站坐标；d: 对应 3 个测量距离(米)；
 * out: 解算标签坐标。返回 0 成功。 */
int UWB_Trilaterate2D(const Point2D* anchors, const float* d, Point2D* out);

#endif /* UWB_2D_H */
