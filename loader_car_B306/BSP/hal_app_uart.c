/**
 * @file    hal_app_uart.c
 * @brief   应用层串口初始化 — 精简版
 *
 * 原来的app_uart_run()中的指令处理逻辑已经移到hal_app_task.c中
 * 此文件只负责串口硬件层面的初始化
 */

#include "hal_app_uart.h"

void app_uart_init(void)
{
    /* UART硬件初始化由CubeMX的 MX_USARTx_UART_Init() 完成 */
    /* 使能各串口RXNE中断 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}
