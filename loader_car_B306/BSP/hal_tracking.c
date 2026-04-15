/**
 * @file    hal_tracking.c
 * @brief   4路红外循迹传感器驱动 — HAL库版本
 *
 * 传感器输出：检测到黑线=1，白地=0
 * 排列顺序（从左到右）：X4, X3, X2, X1
 *
 * 移植要点：
 *   GPIO 初始化由 CubeMX 生成（浮空输入），
 *   本文件仅保留读取宏定义，初始化函数为空壳兼容调用
 */

#include "hal_tracking.h"

/**
 * @brief  循迹传感器初始化
 * @note   GPIO 由 CubeMX 中配置为 Input Floating，MX_GPIO_Init() 自动完成
 *         此函数保留以兼容 app_sensor_init() 的调用
 */
void TRACK_IR4_Init(void)
{
    /* GPIO 已由 CubeMX MX_GPIO_Init() 初始化，此处无需额外操作 */
}
