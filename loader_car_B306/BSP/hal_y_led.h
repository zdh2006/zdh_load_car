#ifndef HAL_Y_LED_H
#define HAL_Y_LED_H

#include "stm32f1xx_hal.h"

/* ---- LED 引脚定义（根据实际硬件修改）---- */
#define LED_GPIO_PORT   GPIOC
#define LED_PIN         GPIO_PIN_13

/* ---- 操作宏 ---- */
#define LED_ON()      HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_OFF()     HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_TOGGLE()  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN)

void led_init(void);

#endif /* HAL_Y_LED_H */
