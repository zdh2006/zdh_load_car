#ifndef HAL_Y_SERVO_H
#define HAL_Y_SERVO_H

#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdlib.h>

/* ---- 舵机数量 ---- */
#define SERVO_NUM   6

/* ---- 舵机引脚定义（根据实际硬件修改）----
   原工程引脚定义在 y_servo.h 中，此处填入实际使用的引脚 */
#define SERVO0_GPIO_PORT    GPIOB
#define SERVO0_PIN          GPIO_PIN_0
#define SERVO1_GPIO_PORT    GPIOB
#define SERVO1_PIN          GPIO_PIN_1
#define SERVO2_GPIO_PORT    GPIOB
#define SERVO2_PIN          GPIO_PIN_2
#define SERVO3_GPIO_PORT    GPIOB
#define SERVO3_PIN          GPIO_PIN_4   /* JTAG禁用后可用 */
#define SERVO4_GPIO_PORT    GPIOB
#define SERVO4_PIN          GPIO_PIN_5
#define SERVO5_GPIO_PORT    GPIOB
#define SERVO5_PIN          GPIO_PIN_8

/* ---- 舵机数据结构 ---- */
typedef struct {
    float   aim;        /* 目标 PWM 宽度 (us) */
    float   current;    /* 当前 PWM 宽度 (us) */
    float   increment;  /* 每个周期增量 */
    int32_t time;       /* 运动时间 (ms) */
    int     bias;       /* 安装偏差补偿 (us) */
} pwmServo_t;

extern pwmServo_t pwmServo_angle[SERVO_NUM];

void pwmServo_init(void);
void pwmServo_angle_set(uint8_t index, int aim, int time);
void pwmServo_stop_motion(uint8_t index);
void pwmServo_bias_set(uint8_t index, int bias);

#endif /* HAL_Y_SERVO_H */
