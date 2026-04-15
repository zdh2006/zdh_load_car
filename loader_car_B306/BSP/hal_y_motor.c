/**
 * @file    hal_y_motor.c
 * @brief   麦轮底盘电机驱动 — HAL库版本
 * @note    使用 TIM8 四通道 PWM 驱动四个直流电机
 *          TIM8 CH1→电机C(PC6), CH2→电机A(PC7), CH3→电机D(PC8), CH4→电机B(PC9)
 *          方向引脚: 电机A→PA11, 电机B→PC10, 电机C→PA8, 电机D→PA12
 *
 *  移植要点（对比标准库）:
 *  1. motor_init() 中 TIM8 PWM 启动改为 HAL_TIM_PWM_Start()
 *     高级定时器会在 HAL_TIM_PWM_Start 内部自动使能 MOE，无需手动 TIM_CtrlPWMOutputs
 *  2. SetSpeed 中 TIM_SetCompareX() 改为 __HAL_TIM_SET_COMPARE()
 *  3. GPIO 操作改为 HAL_GPIO_WritePin()
 *  4. htim8 句柄由 CubeMX 在 tim.c 中生成，此处 extern 引入
 */

#include "hal_y_motor.h"
#include "tim.h"   /* CubeMX 生成的 TIM8 句柄 htim8 */

/* ---- PID 参数（可在运行时调整）---- */
int16_t motor_kp = 800;
int16_t motor_kd = 400;

/* ---- 四轮数据结构体 ---- */
ROBOT_Wheel Wheel_A, Wheel_B, Wheel_C, Wheel_D;

/**
 * @brief  电机初始化：启动 TIM8 四路 PWM 输出
 * @note   CubeMX 已生成 MX_TIM8_Init()，此处只需启动 PWM
 *         在 main.c 中调用 MX_TIM8_Init() 之后调用本函数
 */
void motor_init(void)
{
    /* 启动四路 PWM 输出（高级定时器 HAL_TIM_PWM_Start 内部自动使能 MOE） */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  /* PC6 — 电机C */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  /* PC7 — 电机A */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);  /* PC8 — 电机D */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);  /* PC9 — 电机B */

    /* 初始 PWM 值置 0，确保电机不动 */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
}

/* ========== PID 控制器（四轮各自独立的静态变量）========== */

/**
 * @brief  电机A PD 速度控制器
 * @param  spd_target  目标速度（m/s），已取反（左轮安装方向相反）
 * @param  spd_current 当前实测速度（m/s）
 * @return PWM 输出值，范围 -2000 ~ +2000
 */
int16_t SPEED_PidCtlA(float spd_target, float spd_current)
{
    static int16_t motor_pwm_out = 0;
    static float   bias_last = 0.0f;
    float bias;

    bias = spd_target - spd_current;
    motor_pwm_out += (int16_t)(motor_kp * bias + motor_kd * (bias - bias_last));
    bias_last = bias;

    if (motor_pwm_out >  2000) motor_pwm_out =  2000;
    if (motor_pwm_out < -2000) motor_pwm_out = -2000;
    return motor_pwm_out;
}

/** @brief 电机B PD 控制器（参数同A，独立静态变量） */
int16_t SPEED_PidCtlB(float spd_target, float spd_current)
{
    static int16_t motor_pwm_out = 0;
    static float   bias_last = 0.0f;
    float bias;

    bias = spd_target - spd_current;
    motor_pwm_out += (int16_t)(motor_kp * bias + motor_kd * (bias - bias_last));
    bias_last = bias;

    if (motor_pwm_out >  2000) motor_pwm_out =  2000;
    if (motor_pwm_out < -2000) motor_pwm_out = -2000;
    return motor_pwm_out;
}

/** @brief 电机C PD 控制器 */
int16_t SPEED_PidCtlC(float spd_target, float spd_current)
{
    static int16_t motor_pwm_out = 0;
    static float   bias_last = 0.0f;
    float bias;

    bias = spd_target - spd_current;
    motor_pwm_out += (int16_t)(motor_kp * bias + motor_kd * (bias - bias_last));
    bias_last = bias;

    if (motor_pwm_out >  2000) motor_pwm_out =  2000;
    if (motor_pwm_out < -2000) motor_pwm_out = -2000;
    return motor_pwm_out;
}

/** @brief 电机D PD 控制器 */
int16_t SPEED_PidCtlD(float spd_target, float spd_current)
{
    static int16_t motor_pwm_out = 0;
    static float   bias_last = 0.0f;
    float bias;

    bias = spd_target - spd_current;
    motor_pwm_out += (int16_t)(motor_kp * bias + motor_kd * (bias - bias_last));
    bias_last = bias;

    if (motor_pwm_out >  2000) motor_pwm_out =  2000;
    if (motor_pwm_out < -2000) motor_pwm_out = -2000;
    return motor_pwm_out;
}

/* ========== 电机 PWM 速度控制 ========== */

/**
 * @brief  设置电机A速度
 * @param  speed  -2000 ~ +2000，正值正转，负值反转
 * @note   TIM8_CH2 (PC7) + 方向引脚 PA11
 */
void MOTOR_A_SetSpeed(int16_t speed)
{
    int16_t temp = speed;
    if (temp >  2000) temp =  2000;
    if (temp < -2000) temp = -2000;

    if (temp >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)temp);
        HAL_GPIO_WritePin(MOTOR_A_DIR_GPIO_PORT, MOTOR_A_DIR_PIN, GPIO_PIN_RESET);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)(2000 + temp));
        HAL_GPIO_WritePin(MOTOR_A_DIR_GPIO_PORT, MOTOR_A_DIR_PIN, GPIO_PIN_SET);
    }
}

/**
 * @brief  设置电机B速度
 * @note   TIM8_CH4 (PC9) + 方向引脚 PC10
 */
void MOTOR_B_SetSpeed(int16_t speed)
{
    int16_t temp = speed;
    if (temp >  2000) temp =  2000;
    if (temp < -2000) temp = -2000;

    if (temp >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)temp);
        HAL_GPIO_WritePin(MOTOR_B_DIR_GPIO_PORT, MOTOR_B_DIR_PIN, GPIO_PIN_RESET);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)(2000 + temp));
        HAL_GPIO_WritePin(MOTOR_B_DIR_GPIO_PORT, MOTOR_B_DIR_PIN, GPIO_PIN_SET);
    }
}

/**
 * @brief  设置电机C速度
 * @note   TIM8_CH1 (PC6) + 方向引脚 PA8
 */
void MOTOR_C_SetSpeed(int16_t speed)
{
    int16_t temp = speed;
    if (temp >  2000) temp =  2000;
    if (temp < -2000) temp = -2000;

    if (temp >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)temp);
        HAL_GPIO_WritePin(MOTOR_C_DIR_GPIO_PORT, MOTOR_C_DIR_PIN, GPIO_PIN_RESET);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)(2000 + temp));
        HAL_GPIO_WritePin(MOTOR_C_DIR_GPIO_PORT, MOTOR_C_DIR_PIN, GPIO_PIN_SET);
    }
}

/**
 * @brief  设置电机D速度
 * @note   TIM8_CH3 (PC8) + 方向引脚 PA12
 */
void MOTOR_D_SetSpeed(int16_t speed)
{
    int16_t temp = speed;
    if (temp >  2000) temp =  2000;
    if (temp < -2000) temp = -2000;

    if (temp >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)temp);
        HAL_GPIO_WritePin(MOTOR_D_DIR_GPIO_PORT, MOTOR_D_DIR_PIN, GPIO_PIN_RESET);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)(2000 + temp));
        HAL_GPIO_WritePin(MOTOR_D_DIR_GPIO_PORT, MOTOR_D_DIR_PIN, GPIO_PIN_SET);
    }
}
