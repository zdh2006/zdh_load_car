#ifndef HAL_ULTRASONIC_H
#define HAL_ULTRASONIC_H

#include "stm32f1xx_hal.h"

/* ---- 超声波引脚定义（根据实际硬件修改）---- */
#define TRIG_GPIO_PORT  GPIOA
#define TRIG_PIN        GPIO_PIN_4

#define ECHO_GPIO_PORT  GPIOA
#define ECHO_PIN        GPIO_PIN_5

void  ultrasonic_sensor_init(void);
float ultrasonic_distance_read(void);

#endif /* HAL_ULTRASONIC_H */
