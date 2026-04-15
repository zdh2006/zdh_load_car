#ifndef HAL_Y_DELAY_H
#define HAL_Y_DELAY_H

#include "stm32f1xx_hal.h"

void delay_ms(uint16_t delay_ms_val);
void delay_us(uint16_t delay_us_val);
void delay_ns(uint16_t t);

#endif /* HAL_Y_DELAY_H */
