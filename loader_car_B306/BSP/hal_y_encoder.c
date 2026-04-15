/**
 * @file    hal_y_encoder.c
 * @brief   四路编码器驱动 — HAL库版本
 *
 * 编码器与定时器对应关系：
 *   TIM2（全重映射）：PA15(CH1) + PB3(CH2) → 电机D
 *   TIM3：PA6(CH1)  + PA7(CH2)  → 电机A
 *   TIM4：PB6(CH1)  + PB7(CH2)  → 电机B
 *   TIM5：PA0(CH1)  + PA1(CH2)  → 电机C
 *
 * 移植要点：
 *   1. CubeMX 将各 TIM 配置为 Encoder Mode，自动生成 MX_TIMx_Init()
 *   2. Encoder_Init() 改为调用 HAL_TIM_Encoder_Start() 启动捕获
 *   3. 读取计数器改为 __HAL_TIM_GET_COUNTER()，清零直接写 Instance->CNT
 *   4. TIM2 PA15/PB3 引脚重映射：CubeMX 在 TIM2 GPIO Settings 中手动选择
 *      CH1=PA15，CH2=PB3（Full Remap），需同时在 SYS 中禁用 JTAG
 */

#include "hal_y_encoder.h"
#include "tim.h"   /* CubeMX 生成的 htim2/3/4/5 句柄 */

/**
 * @brief  初始化并启动全部四路编码器
 * @note   在 MX_TIM2/3/4/5_Init() 之后调用
 */
void Encoder_Init(void)
{
    /* TIM2 全重映射：PA15(CH1) + PB3(CH2) — 电机D */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* TIM3：PA6 + PA7 — 电机A */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* TIM4：PB6 + PB7 — 电机B */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* TIM5：PA0 + PA1 — 电机C */
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

/**
 * @brief  读取电机A编码器增量（TIM3）
 * @return 脉冲增量（带符号 int16_t，自动处理正/反转）
 */
int16_t ENCODER_A_GetCounter(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    htim3.Instance->CNT = 0;
    return cnt;
}

/**
 * @brief  读取电机B编码器增量（TIM4）
 */
int16_t ENCODER_B_GetCounter(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    htim4.Instance->CNT = 0;
    return cnt;
}

/**
 * @brief  读取电机C编码器增量（TIM5）
 */
int16_t ENCODER_C_GetCounter(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
    htim5.Instance->CNT = 0;
    return cnt;
}

/**
 * @brief  读取电机D编码器增量（TIM2，全重映射）
 */
int16_t ENCODER_D_GetCounter(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    htim2.Instance->CNT = 0;
    return cnt;
}
