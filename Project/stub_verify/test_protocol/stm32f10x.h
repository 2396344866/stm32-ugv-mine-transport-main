/* 宿主机（PC）编译用的最小替身头文件。
 * 协议层 bcm_door.h 只依赖整型宽度别名，不依赖任何 STM32 寄存器定义，
 * 因此可用这一个文件把协议层搬到 PC 上真跑一遍（BCM_Door_PackReq/ParseAck）。
 * 仅用于 Project/stub_verify/test_protocol，不参与固件编译。 */
#ifndef STM32F10X_H_NATIVE_TEST
#define STM32F10X_H_NATIVE_TEST

#include <stdint.h>

#endif
