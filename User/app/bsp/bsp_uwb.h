#ifndef BSP_UWB_H
#define BSP_UWB_H

#include "stm32f10x.h"

/* 集成安信可 BU03 UWB 模组（基于 DW3000），主机通过 UART-AT 交互。
 * 默认采用双向测距(TWR)由模块内部完成，主机解析距离结果。 */
void BSP_UWB_Init(void);
int  UWB_StartRanging(void);     /* 启动测距，返回 0 成功 */
int  UWB_ReadDistance(float* dist_m);  /* 解析最新一帧距离(米)，返回 0 成功 */

#endif /* BSP_UWB_H */
