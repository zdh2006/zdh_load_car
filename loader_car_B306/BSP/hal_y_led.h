/**
 * @file    hal_y_led.h
 * @brief   LED驱动 — 空壳版本（未配置LED引脚时使用）
 *
 * 说明: 由于CubeMX中未配置LED引脚, 本文件将所有LED操作改为空操作
 *       保留接口是为了兼容其他文件的调用, 不影响编译
 *       如果以后添加了LED, 只需修改这个头文件即可恢复功能
 */

#ifndef HAL_Y_LED_H
#define HAL_Y_LED_H

#include "stm32f1xx_hal.h"

/* ---- LED操作宏: 空操作(不做任何事) ----
   如果以后添加LED, 把这里改回真实的GPIO操作即可:
   例如: #define LED_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
*/
#define LED_ON()      ((void)0)
#define LED_OFF()     ((void)0)
#define LED_TOGGLE()  ((void)0)

void led_init(void);

#endif /* HAL_Y_LED_H */