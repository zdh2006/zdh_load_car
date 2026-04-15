#ifndef HAL_Y_ENCODER_H
#define HAL_Y_ENCODER_H

#include "stm32f1xx_hal.h"

void    Encoder_Init(void);
int16_t ENCODER_A_GetCounter(void);
int16_t ENCODER_B_GetCounter(void);
int16_t ENCODER_C_GetCounter(void);
int16_t ENCODER_D_GetCounter(void);

#endif /* HAL_Y_ENCODER_H */
