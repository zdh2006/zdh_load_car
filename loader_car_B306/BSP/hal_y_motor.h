#ifndef HAL_Y_MOTOR_H
#define HAL_Y_MOTOR_H

#include "stm32f1xx_hal.h"

/* ---- 电机方向引脚定义（根据实际硬件修改）---- */
#define MOTOR_A_DIR_GPIO_PORT   GPIOA
#define MOTOR_A_DIR_PIN         GPIO_PIN_11

#define MOTOR_B_DIR_GPIO_PORT   GPIOC
#define MOTOR_B_DIR_PIN         GPIO_PIN_10

#define MOTOR_C_DIR_GPIO_PORT   GPIOA
#define MOTOR_C_DIR_PIN         GPIO_PIN_8

#define MOTOR_D_DIR_GPIO_PORT   GPIOA
#define MOTOR_D_DIR_PIN         GPIO_PIN_12

/* ---- 编码器换算系数（根据轮径和减速比标定）---- */
/* 电机PPR × 减速比 × 2（双边沿）× 4（正交） = 每转脉冲数 */
/* MEC_WHEEL_SCALE = 轮周长(m) / 每转脉冲数 / 定时周期(s) */
/* 此处为示例值，需根据实际硬件测量 */
#define MEC_WHEEL_SCALE  0.000035f

/* ---- 车轮数据结构 ---- */
typedef struct {
    float   TG;     /* 目标速度 (m/s) */
    float   RT;     /* 实时速度 (m/s)，由编码器计算 */
    int16_t PWM;    /* PID 输出 PWM 值 */
} ROBOT_Wheel;

extern ROBOT_Wheel Wheel_A, Wheel_B, Wheel_C, Wheel_D;
extern int16_t motor_kp, motor_kd;

void    motor_init(void);
int16_t SPEED_PidCtlA(float spd_target, float spd_current);
int16_t SPEED_PidCtlB(float spd_target, float spd_current);
int16_t SPEED_PidCtlC(float spd_target, float spd_current);
int16_t SPEED_PidCtlD(float spd_target, float spd_current);
void    MOTOR_A_SetSpeed(int16_t speed);
void    MOTOR_B_SetSpeed(int16_t speed);
void    MOTOR_C_SetSpeed(int16_t speed);
void    MOTOR_D_SetSpeed(int16_t speed);

#endif /* HAL_Y_MOTOR_H */
