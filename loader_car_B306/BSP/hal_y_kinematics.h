#ifndef HAL_Y_KINEMATICS_H
#define HAL_Y_KINEMATICS_H

#include "stm32f1xx_hal.h"

typedef struct {
    float L0, L1, L2, L3;          /* 连杆长度（×10存储，单位 mm×10）*/
    float servo_angle[4];           /* 各关节角度 (°) */
    int   servo_pwm[4];             /* 各关节 PWM 值 (us) */
} kinematics_t;

extern kinematics_t kinematics;

void setup_kinematics(float L0, float L1, float L2, float L3, kinematics_t *k);
int  kinematics_analysis(float x, float y, float z, float Alpha, kinematics_t *k);

#endif /* HAL_Y_KINEMATICS_H */
