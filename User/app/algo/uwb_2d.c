/**
  * @file    uwb_2d.c
  * @brief   二维三边定位（Gauss-Newton 最小二乘）
  */
#include "uwb_2d.h"
#include <math.h>

#ifndef M_PI
	#define M_PI 3.14159265358979323846f
#endif

int UWB_Trilaterate2D(const Point2D* anchors, const float* d, Point2D* out)
{
    /* 初始猜测：三个基站的几何中心 */
    float x = (anchors[0].x + anchors[1].x + anchors[2].x) / 3.0f;
    float y = (anchors[0].y + anchors[1].y + anchors[2].y) / 3.0f;

    for (int iter = 0; iter < 16; iter++)
    {
        float sum_xx = 0, sum_xy = 0, sum_yy = 0, sum_xe = 0, sum_ye = 0;

        for (int i = 0; i < 3; i++)
        {
            float dx = x - anchors[i].x;
            float dy = y - anchors[i].y;
            float dist = (float)sqrtf(dx * dx + dy * dy) + 1e-6f;
            float e = dist - d[i];             /* 残差 */

            float gx = dx / dist;
            float gy = dy / dist;

            sum_xx += gx * gx;
            sum_xy += gx * gy;
            sum_yy += gy * gy;
            sum_xe += gx * e;
            sum_ye += gy * e;
        }

        /* 法方程 (2x2) 求解增量 */
        float det = sum_xx * sum_yy - sum_xy * sum_xy;
        if (det < 1e-9f) return -1;
        float dxp = ( sum_yy * sum_xe - sum_xy * sum_ye) / det;
        float dyp = ( sum_xx * sum_ye - sum_xy * sum_xe) / det;

        x -= dxp;
        y -= dyp;

        if ((dxp * dxp + dyp * dyp) < 1e-6f) break;  /* 收敛 */
    }

    out->x = x;
    out->y = y;
    return 0;
}
