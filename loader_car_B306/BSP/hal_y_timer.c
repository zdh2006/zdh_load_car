/**
 * @file    hal_y_timer.c
 * @brief   系统时基与电机PID调度 — HAL库版本
 *
 * 移植要点：
 *   1. SysTick 由 CubeMX/HAL 自动初始化（HAL_Init 内部调用），无需手写 SysTick_Config
 *   2. millis() 直接返回 HAL_GetTick()（HAL库内部维护 uwTick）
 *   3. 每 20ms 执行一次 app_motor_run() 的机制：
 *      重写 HAL_SYSTICK_Callback()，HAL 在每次 SysTick_Handler 中
 *      调用 HAL_IncTick() 后自动回调此函数
 *   4. SysTick_Handler 保留在 stm32f1xx_it.c，无需修改
 *
 * 使用方式：
 *   - 在 main.c 调用 HAL_Init() 后，SysTick 已自动运行
 *   - 直接调用 millis() 获取毫秒时间戳
 *   - HAL_SYSTICK_Callback() 会自动被每 1ms 的 SysTick 中断触发
 */

#include "hal_y_timer.h"
#include "hal_app_motor.h"

/**
 * @brief  获取系统运行毫秒数（等同原工程 millis()）
 * @return 毫秒时间戳（uint32_t）
 */
uint32_t millis(void)
{
    return HAL_GetTick();
}

/**
 * @brief  SysTick 回调（每 1ms 自动调用）
 * @note   此函数替代原工程 SysTick_Handler 中的电机调度逻辑
 *         HAL 在 SysTick_Handler → HAL_IncTick() 之后调用此函数
 *         无需修改 stm32f1xx_it.c 中的 SysTick_Handler
 */
void HAL_SYSTICK_Callback(void)
{
    static uint8_t motor_run_cnt = 0;

    motor_run_cnt++;
    if (motor_run_cnt >= 20)
    {
        motor_run_cnt = 0;
        app_motor_run();   /* 每 20ms 执行一次 PID 闭环控制 */
    }
}
