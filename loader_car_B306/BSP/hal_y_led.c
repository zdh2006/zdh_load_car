/**
 * @file    hal_y_led.c
 * @brief   状态指示 LED 驱动 — HAL库版本
 *
 * 移植要点：
 *   GPIO 初始化由 CubeMX 完成，led_init() 仅兼容原调用接口。
 *   LED_TOGGLE() 改为 HAL_GPIO_TogglePin()。
 */

#include "hal_y_led.h"

/**
 * @brief  LED 初始化（兼容接口）
 * @note   实际 GPIO 初始化由 CubeMX MX_GPIO_Init() 完成
 */
void led_init(void)
{
    /* GPIO 已由 CubeMX 初始化，确保初始为熄灭状态 */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
}
