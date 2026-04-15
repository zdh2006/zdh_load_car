#ifndef HAL_APP_MOTOR_H
#define HAL_APP_MOTOR_H

#include "hal_y_motor.h"
#include "hal_y_encoder.h"

void motor_speed_set(float A, float B, float C, float D);
void app_motor_init(void);
void app_motor_run(void);

#endif /* HAL_APP_MOTOR_H */
