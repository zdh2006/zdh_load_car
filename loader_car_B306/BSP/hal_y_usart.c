/**
 * @file    hal_y_usart.c
 * @brief   四路串口驱动 — HAL库版本
 *
 * 串口配置：
 *   USART1：PA9(TX)  PA10(RX)  115200  全双工  中断接收
 *   USART2：PA2(TX)  PA3(RX)   115200  全双工  中断接收
 *   USART3：PB10(TX) PB11(RX)  115200  全双工  中断接收（OD输出）
 *   UART5： PC12(TX)            115200  半双工  中断接收
 *
 * printf 重定向：→ UART5（HAL_UART_Transmit 阻塞方式）
 *
 * 移植要点：
 *   1. CubeMX 生成 MX_USARTx_UART_Init()，无需手动写初始化代码
 *   2. 中断接收：在 stm32f1xx_it.c 的 USARTx_IRQHandler 中
 *      先读 DR 再调 HAL_UART_IRQHandler（或直接在用户 IRQ 中处理）
 *   3. 本文件提供的 IRQ handler 需替换 stm32f1xx_it.c 中对应的函数
 *      或在 stm32f1xx_it.c 中调用本文件的 uart_data_parse()
 *   4. printf：重写 __io_putchar 或 fputc，使用 HAL_UART_Transmit
 */

#include "hal_y_usart.h"
#include "usart.h"   /* CubeMX 生成的 huart1/2/3/uart5 句柄 */
#include <stdio.h>
#include <string.h>

/* ---- 全局接收缓冲区 ---- */
char     uart_receive_buf[UART_BUF_SIZE];
uint16_t uart_get_ok  = 0;
char     uart_mode    = 0;
uint8_t  uart_receive_num = 0;

/* ---- 发送函数 ---- */

void uart1_send_byte(char dat)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&dat, 1, 100);
}

void uart1_send_str(char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

void uart1_send_int(int tmp)
{
    char str[20];
    snprintf(str, sizeof(str), "%d", tmp);
    uart1_send_str(str);
}

void uart2_send_str(char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

void uart3_send_str(char *s)
{
    /* 发送期间临时关闭 RX 中断，防止数据混乱 */
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
    HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 200);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

void uart5_send_str(char *s)
{
    __HAL_UART_DISABLE_IT(&huart5, UART_IT_RXNE);
    HAL_UART_Transmit(&huart5, (uint8_t *)s, (uint16_t)strlen(s), 200);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
}

/* ---- printf 重定向（使用 HAL_UART_Transmit）---- */
/* 需在工程设置中勾选 Use MicroLIB 或添加半主机重定向 */
int fputc(int ch, FILE *f)
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart5, &c, 1, 100);
    return ch;
}

/* ---- open/close 兼容接口（原工程有引用）---- */
void uart1_open(void)  { __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); }
void uart1_close(void) { __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE); }

/* ========== 统一数据包解析 ========== */
/**
 * @brief  接收字节解析状态机
 * @param  rx_data   接收到的字节
 * @param  uart_num  来源串口号（1/2/3/5）
 *
 * 帧格式：
 *   '$'....'!'   → mode=1 命令帧
 *   '#'....'!'   → mode=2 单舵机帧
 *   '{'.....'}'  → mode=3 多舵机帧
 *   '<'.....'>'  → mode=4 存储帧
 */
void uart_data_parse(char rx_data, uint8_t uart_num)
{
    static uint16_t buf_index = 0;

    if (uart_get_ok) return;   /* 上一帧尚未处理，丢弃新数据 */

    if (uart_mode == 0)
    {
        buf_index = 0;
        switch (rx_data)
        {
            case '$': uart_mode = 1; break;
            case '#': uart_mode = 2; break;
            case '{': uart_mode = 3; break;
            case '<': uart_mode = 4; break;
            default: return;           /* 未识别，不存入缓冲区 */
        }
    }

    if (buf_index < UART_BUF_SIZE - 1)
        uart_receive_buf[buf_index++] = rx_data;

    /* 帧结束判断 */
    if      ((uart_mode == 1) && (rx_data == '!')) { uart_receive_buf[buf_index] = '\0'; uart_get_ok = 1; }
    else if ((uart_mode == 2) && (rx_data == '!')) { uart_receive_buf[buf_index] = '\0'; uart_get_ok = 1; }
    else if ((uart_mode == 3) && (rx_data == '}')) { uart_receive_buf[buf_index] = '\0'; uart_get_ok = 1; }
    else if ((uart_mode == 4) && (rx_data == '>')) { uart_receive_buf[buf_index] = '\0'; uart_get_ok = 1; }

    if (uart_get_ok)
        uart_receive_num = uart_num;

    if (buf_index >= UART_BUF_SIZE)
        buf_index = 0;
}

/* ========== 中断服务程序 ==========
 * 这些函数写在 stm32f1xx_it.c 中（CubeMX 已生成框架），
 * 在用户代码段中添加以下逻辑，或直接将本文件中的实现粘贴到 it.c。
 *
 * 注意：HAL_UART_IRQHandler 会自动清除 RXNE 标志，
 *       若直接读 DR 需手动清标志。本文件采用"先读DR再调HAL"方式。
 */

