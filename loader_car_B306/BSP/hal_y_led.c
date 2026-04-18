/**
 * @file    hal_y_led.c
 * @brief   LED驱动 — 空实现版本
 *
 * 由于未配置LED引脚, led_init() 为空函数
 * 其他文件调用 LED_ON/OFF/TOGGLE 宏时不会实际操作GPIO
 */

#include "hal_y_led.h"

void led_init(void)
{
    /* 未配置LED引脚, 此函数为空 */
    /* 如果以后添加LED, 在这里初始化为熄灭状态:
       HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
    */
}