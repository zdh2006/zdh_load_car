#ifndef HAL_TRACKING_H
#define HAL_TRACKING_H

#include "stm32f1xx_hal.h"

/* ---- 循迹传感器引脚定义（根据实际硬件修改）---- */
/* X1: 最右侧，X4: 最左侧 */
#define TRTACK_IR4_X1_PORT   GPIOC
#define TRTACK_IR4_X1_PIN    GPIO_PIN_0

#define TRTACK_IR4_X2_PORT   GPIOC
#define TRTACK_IR4_X2_PIN    GPIO_PIN_1

#define TRTACK_IR4_X3_PORT   GPIOC
#define TRTACK_IR4_X3_PIN    GPIO_PIN_2

#define TRTACK_IR4_X4_PORT   GPIOC
#define TRTACK_IR4_X4_PIN    GPIO_PIN_3

/* ---- 读取宏（返回 0 或 1）---- */
#define TRTACK_IR4_X1_READ()  ((uint8_t)HAL_GPIO_ReadPin(TRTACK_IR4_X1_PORT, TRTACK_IR4_X1_PIN))
#define TRTACK_IR4_X2_READ()  ((uint8_t)HAL_GPIO_ReadPin(TRTACK_IR4_X2_PORT, TRTACK_IR4_X2_PIN))
#define TRTACK_IR4_X3_READ()  ((uint8_t)HAL_GPIO_ReadPin(TRTACK_IR4_X3_PORT, TRTACK_IR4_X3_PIN))
#define TRTACK_IR4_X4_READ()  ((uint8_t)HAL_GPIO_ReadPin(TRTACK_IR4_X4_PORT, TRTACK_IR4_X4_PIN))

void TRACK_IR4_Init(void);

#endif /* HAL_TRACKING_H */
