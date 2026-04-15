/**
 * @file    hal_app_uart.c
 * @brief   应用层串口指令处理 — HAL库版本
 *
 * 与原工程逻辑完全一致。
 * uart_get_ok 由串口中断置 1，主循环调用 app_uart_run() 处理。
 *
 * 指令模式说明：
 *   mode=1  '$'...'!'    命令帧，调用 parse_cmd()
 *   mode=2  '#'...'!'    单舵机帧，调用 parse_action()
 *   mode=3  '{'...'}'    多舵机帧，调用 parse_action()
 *   mode=4  '<'...'>'    存储帧，调用 save_action()
 */

#include "hal_app_uart.h"

/**
 * @brief  串口模块初始化
 * @note   UART 硬件初始化由 CubeMX MX_USARTx_UART_Init() 完成，
 *         此处仅作兼容接口（可以为空，也可加入业务初始化）
 */
void app_uart_init(void)
{
    /* 硬件初始化已由 CubeMX 生成的 MX_USARTx_UART_Init() 完成 */
    /* 使能各串口 RXNE 中断（CubeMX 若已勾选中断则自动使能，否则在此手动使能）*/
    // __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    // __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    // __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
    // __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
}

/**
 * @brief  串口指令处理主循环（放入 while(1) 中调用）
 * @note   uart_get_ok 由 uart_data_parse() 在中断中置 1，
 *         此函数在主循环中轮询处理，处理完后清零
 */
void app_uart_run(void)
{
    // if (!uart_get_ok) return;

    // switch (uart_mode)
    // {
    //     case 1:
    //         /* 命令模式：$XXX! */
    //         parse_cmd(uart_receive_buf);
    //         break;

    //     case 2:
    //         /* 单舵机调试：#000P1500T1000! */
    //         parse_action(uart_receive_buf);
    //         break;

    //     case 3:
    //         /* 多舵机调试：{#000P1500T1000!#001P1500T1000!} */
    //         parse_action(uart_receive_buf);
    //         break;

    //     case 4:
    //         /* 存储动作组：<G0000#000P1500T1000!> */
    //         save_action(uart_receive_buf);
    //         break;

    //     default:
    //         break;
    // }

    uart_get_ok = 0;
    uart_mode   = 0;
}
