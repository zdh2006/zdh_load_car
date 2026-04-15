/**
 * @file    hal_app_motor.c
 * @brief   应用层电机控制 — HAL库版本
 *
 * 与原工程逻辑完全一致，仅头文件引用改为HAL版本。
 * app_motor_run() 由 HAL_SYSTICK_Callback() 每 20ms 调用一次。
 */

#include "hal_app_motor.h"

/**
 * @brief  设置四轮目标速度
 * @param  A/B/C/D  各轮目标速度（m/s），正值前进方向
 */
void motor_speed_set(float A, float B, float C, float D)
{
    Wheel_A.TG = A;
    Wheel_B.TG = B;
    Wheel_C.TG = C;
    Wheel_D.TG = D;
}

/**
 * @brief  电机模块初始化
 * @note   在 main() 中 MX_TIM8_Init() 和 MX_TIMx_Init() 之后调用
 */
void app_motor_init(void)
{
    motor_init();
    Encoder_Init();
}

/**
 * @brief  电机 PID 闭环控制（每 20ms 执行一次）
 * @note   由 hal_y_timer.c 的 HAL_SYSTICK_Callback() 自动调度，
 *         不要在主循环中直接调用
 *
 * 执行流程：
 *   1. 读取编码器脉冲增量，换算为实时速度 (m/s)
 *   2. PD 控制器计算 PWM 输出值
 *   3. 调用 SetSpeed 写入 TIM8 比较寄存器
 *
 * 注意：左侧电机（A/C）安装方向相反，PID 输入和 SetSpeed 均取反
 */
void app_motor_run(void)
{
    /* Step 1: 读取编码器，计算实时速度 */
    Wheel_A.RT = (float)(ENCODER_A_GetCounter()) * MEC_WHEEL_SCALE;
    Wheel_B.RT = (float)(ENCODER_B_GetCounter()) * MEC_WHEEL_SCALE;
    Wheel_C.RT = (float)(ENCODER_C_GetCounter()) * MEC_WHEEL_SCALE;
    Wheel_D.RT = (float)(ENCODER_D_GetCounter()) * MEC_WHEEL_SCALE;

    /* Step 2: PID 计算
     * A/C 为左侧轮（安装反向），TG 取反后传入 PID，使误差方向一致 */
    Wheel_A.PWM = SPEED_PidCtlA(-Wheel_A.TG, Wheel_A.RT);  /* L1 */
    Wheel_B.PWM = SPEED_PidCtlB( Wheel_B.TG, Wheel_B.RT);  /* R1 */
    Wheel_C.PWM = SPEED_PidCtlC(-Wheel_C.TG, Wheel_C.RT);  /* L2 */
    Wheel_D.PWM = SPEED_PidCtlD( Wheel_D.TG, Wheel_D.RT);  /* R2 */

    /* Step 3: 输出 PWM（SetSpeed 内部再次取反，补偿左轮方向）*/
    MOTOR_A_SetSpeed(-Wheel_A.PWM);
    MOTOR_B_SetSpeed(-Wheel_B.PWM);
    MOTOR_C_SetSpeed(-Wheel_C.PWM);
    MOTOR_D_SetSpeed(-Wheel_D.PWM);
}
