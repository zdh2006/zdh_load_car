/**
 * @file    hal_ultrasonic.c
 * @brief   HC-SR04 超声波测距 — HAL库版本
 *
 * 使用 TIM6 计时（1us 精度），计算高电平持续时间换算距离。
 *
 * 移植要点：
 *   1. TIM6 初始化由 CubeMX 生成（Prescaler=71，ARR=30000）
 *   2. 启动/停止用 HAL_TIM_Base_Start / HAL_TIM_Base_Stop
 *   3. 读取计数器：__HAL_TIM_GET_COUNTER()
 *   4. 清零计数器：__HAL_TIM_SET_COUNTER()
 *   5. GPIO：HAL_GPIO_WritePin / HAL_GPIO_ReadPin
 *   6. millis() 使用 HAL_GetTick()（定义在 hal_y_timer.c）
 */

#include "hal_ultrasonic.h"
#include "tim.h"
#include "hal_y_delay.h"

/**
 * @brief  超声波传感器初始化
 * @note   GPIO 由 CubeMX 初始化，TIM6 由 MX_TIM6_Init() 初始化
 *         此处仅确保 Trig 引脚初始为低
 */
void ultrasonic_sensor_init(void)
{
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  读取超声波距离
 * @return 距离（cm），超时或无回波返回 -1
 *
 * 时序：
 *   1. Trig 拉高 ≥10us 触发脉冲
 *   2. 等待 Echo 拉高（高电平开始计时）
 *   3. 等待 Echo 拉低（停止计时）
 *   4. 距离 = 高电平时间(us) × 0.017 cm
 */
float ultrasonic_distance_read(void)
{
    uint32_t start_time;
    uint16_t csb_t;
    const uint32_t timeout_ms = 20U;

    /* Step 1: 触发脉冲（≥10us）*/
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_PIN, GPIO_PIN_RESET);

    /* Step 2: 等待 Echo 高电平 */
    start_time = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ECHO_GPIO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - start_time) > timeout_ms)
            return -1.0f;
    }

    /* Step 3: 启动 TIM6 计时 */
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_TIM_Base_Start(&htim6);

    start_time = HAL_GetTick();
    while (HAL_GPIO_ReadPin(ECHO_GPIO_PORT, ECHO_PIN) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start_time) > timeout_ms)
        {
            HAL_TIM_Base_Stop(&htim6);
            return -1.0f;
        }
    }

    csb_t = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);
    HAL_TIM_Base_Stop(&htim6);

    /* Step 4: 计算距离 */
    if (csb_t < 25000U)
        return (float)csb_t * 0.017f;

    return -1.0f;
}
