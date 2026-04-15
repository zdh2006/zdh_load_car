/**
 * @file    hal_y_servo.c
 * @brief   6路舵机软件PWM驱动 — HAL库版本
 *
 * 原理：TIM7 基本定时器中断，每次中断交替将引脚置高/低。
 *       高电平持续时间 = current 值（500~2500us）
 *       低电平持续时间 = 2500 - current
 *       8路轮询，每路 2.5ms，合计 20ms 周期
 *
 * 移植要点：
 *   1. CubeMX 配置 TIM7：Prescaler=71，Period=9，使能全局中断
 *   2. TIM7_IRQHandler 在 stm32f1xx_it.c 中调用 HAL_TIM_IRQHandler(&htim7)
 *      HAL 内部触发 HAL_TIM_PeriodElapsedCallback()，在此实现舵机逻辑
 *   3. ARR 动态修改：__HAL_TIM_SET_AUTORELOAD() 替代 TIM7->ARR 直写
 *   4. GPIO 操作：HAL_GPIO_WritePin() 替代 GPIO_WriteBit()
 */

#include "hal_y_servo.h"
#include "tim.h"   /* htim7 句柄 */

pwmServo_t pwmServo_angle[SERVO_NUM];

/* ---- 私有函数声明 ---- */
static void servo_increment_offset(uint8_t index);
static void servo_pin_set(uint8_t index, GPIO_PinState level);

/**
 * @brief  舵机初始化：配置 GPIO，启动 TIM7 中断
 * @note   CubeMX 已生成 MX_TIM7_Init()，此处只需启动中断
 */
void pwmServo_init(void)
{
    uint8_t i;

    /* 初始化所有舵机状态 */
    for (i = 0; i < SERVO_NUM; i++)
    {
        pwmServo_angle[i].aim       = 1500;
        pwmServo_angle[i].current   = 1500;
        pwmServo_angle[i].increment = 0.0f;
        pwmServo_angle[i].time      = 5000;
        pwmServo_angle[i].bias      = 0;
    }

    /* 启动 TIM7 基本定时器中断 */
    HAL_TIM_Base_Start_IT(&htim7);
}

/**
 * @brief  设置舵机目标位置和运动时间
 * @param  index  舵机编号 0~(SERVO_NUM-1)，255=全部
 * @param  aim    目标 PWM 宽度（500~2500 us）
 * @param  time   运动时间（ms），<20 则直接到位
 */
void pwmServo_angle_set(uint8_t index, int aim, int time)
{
    uint8_t i;

    if (aim > 2500 || aim < 500) return;

    if (index == 255)
    {
        for (i = 0; i < SERVO_NUM; i++)
        {
            pwmServo_angle[i].aim       = aim;
            pwmServo_angle[i].time      = time;
            pwmServo_angle[i].increment = (float)(aim - pwmServo_angle[i].current) / (time / 20.0f);
        }
        return;
    }

    if (index >= SERVO_NUM) return;
    if (time > 10000) time = 10000;

    if (time < 20)
    {
        pwmServo_angle[index].aim       = aim;
        pwmServo_angle[index].current   = aim;
        pwmServo_angle[index].increment = 0.0f;
    }
    else
    {
        pwmServo_angle[index].aim       = aim;
        pwmServo_angle[index].time      = time;
        pwmServo_angle[index].increment = (float)(aim - pwmServo_angle[index].current) / (time / 20.0f);
    }
}

/**
 * @brief  停止指定舵机运动
 * @param  index  舵机编号，255=全部
 */
void pwmServo_stop_motion(uint8_t index)
{
    uint8_t i;
    if (index == 255)
    {
        for (i = 0; i < SERVO_NUM; i++)
        {
            pwmServo_angle[i].aim       = pwmServo_angle[i].current;
            pwmServo_angle[i].increment = 0.001f;
        }
        return;
    }
    if (index >= SERVO_NUM) return;
    pwmServo_angle[index].aim       = pwmServo_angle[index].current;
    pwmServo_angle[index].increment = 0.001f;
}

/**
 * @brief  设置舵机安装偏差补偿
 * @param  index  舵机编号
 * @param  bias   偏差量（us），加在 aim 基础上
 */
void pwmServo_bias_set(uint8_t index, int bias)
{
    if (index >= SERVO_NUM) return;
    pwmServo_angle[index].bias      = bias;
    pwmServo_angle[index].increment = 0.001f;
}

/* ---- 私有：更新舵机当前位置 ---- */
static void servo_increment_offset(uint8_t index)
{
    int aim_temp;

    if (pwmServo_angle[index].increment == 0.0f) return;

    aim_temp = pwmServo_angle[index].aim + pwmServo_angle[index].bias;
    if (aim_temp > 2490) aim_temp = 2490;
    if (aim_temp < 500)  aim_temp = 500;

    if (abs((int)(aim_temp - pwmServo_angle[index].current)) <=
        (int)fabsf(pwmServo_angle[index].increment * 2))
    {
        pwmServo_angle[index].current   = (float)aim_temp;
        pwmServo_angle[index].increment = 0.0f;
    }
    else
    {
        pwmServo_angle[index].current += pwmServo_angle[index].increment;
    }
}

/* ---- 私有：设置舵机引脚电平 ---- */
static void servo_pin_set(uint8_t index, GPIO_PinState level)
{
    switch (index)
    {
        case 0: HAL_GPIO_WritePin(SERVO0_GPIO_PORT, SERVO0_PIN, level); break;
        case 1: HAL_GPIO_WritePin(SERVO1_GPIO_PORT, SERVO1_PIN, level); break;
        case 2: HAL_GPIO_WritePin(SERVO2_GPIO_PORT, SERVO2_PIN, level); break;
        case 3: HAL_GPIO_WritePin(SERVO3_GPIO_PORT, SERVO3_PIN, level); break;
        case 4: HAL_GPIO_WritePin(SERVO4_GPIO_PORT, SERVO4_PIN, level); break;
        case 5: HAL_GPIO_WritePin(SERVO5_GPIO_PORT, SERVO5_PIN, level); break;
        default: break;
    }
}

/**
 * @brief  TIM7 周期中断回调（软件PWM核心）
 * @note   由 HAL_TIM_IRQHandler() 调用，每次中断交替：
 *         flag=0：设引脚高，ARR=current（高电平持续时间 us）
 *         flag=1：设引脚低，ARR=2500-current（低电平持续时间 us）
 *         8路轮询完成一个 20ms 周期
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM7) return;

    static uint8_t flag = 0;
    static uint8_t servo_index = 0;
    uint32_t low_time;

    if (servo_index >= SERVO_NUM)
        servo_index = 0;

    if (flag == 0)
    {
        /* 设引脚高，ARR 设为高电平持续时间 */
        __HAL_TIM_SET_AUTORELOAD(&htim7, (uint32_t)pwmServo_angle[servo_index].current);
        servo_pin_set(servo_index, GPIO_PIN_SET);
        servo_increment_offset(servo_index);
    }
    else
    {
        /* 设引脚低，ARR 设为低电平持续时间，切换到下一路 */
        low_time = 2500U - (uint32_t)pwmServo_angle[servo_index].current;
        __HAL_TIM_SET_AUTORELOAD(&htim7, low_time);
        servo_pin_set(servo_index, GPIO_PIN_RESET);
        servo_index++;
    }
    flag ^= 1;
}
