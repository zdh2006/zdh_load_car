/**
 * @file    hal_y_delay.c
 * @brief   延时函数 — HAL库版本
 *
 * delay_ms 改为 HAL_Delay（精度1ms，基于SysTick）
 * delay_us 保留软件循环（HAL无微秒延时，可用DWT替代）
 */

#include "hal_y_delay.h"

/**
 * @brief  毫秒级延时（基于 HAL_Delay）
 * @param  delay_ms  延时毫秒数
 * @note   HAL_Delay 基于 SysTick，最小 1ms，可在非中断上下文使用
 *         在中断中调用会死锁，请改用 millis() 轮询方式
 */
void delay_ms(uint16_t delay_ms_val)
{
    HAL_Delay(delay_ms_val);
}

/**
 * @brief  微秒级软件延时（基于 NOP 循环，精度受编译优化影响）
 * @param  delay_us  延时微秒数（@72MHz，每次循环约 1us）
 * @note   更精确的 us 延时可启用 DWT 计数器：
 *         CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
 *         DWT->CYCCNT = 0;
 *         DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
 *         然后轮询 DWT->CYCCNT
 */
void delay_us(uint16_t delay_us_val)
{
    volatile uint32_t num;
    volatile uint32_t t;
    for (num = 0; num < delay_us_val; num++)
    {
        t = 11;
        while (t--);
    }
}

void delay_ns(uint16_t t)
{
    while (t--);
}
