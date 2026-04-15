#ifndef HAL_Y_USART_H
#define HAL_Y_USART_H

#include "stm32f1xx_hal.h"
#include <stdio.h>

#define UART_BUF_SIZE  256

extern char     uart_receive_buf[UART_BUF_SIZE];
extern uint16_t uart_get_ok;
extern char     uart_mode;
extern uint8_t  uart_receive_num;

/* 发送接口 */
void uart1_send_byte(char dat);
void uart1_send_str(char *s);
void uart1_send_int(int tmp);
void uart2_send_str(char *s);
void uart3_send_str(char *s);
void uart5_send_str(char *s);

/* 使能/禁用接口（兼容原工程调用） */
void uart1_open(void);
void uart1_close(void);

/* 数据解析 */
void uart_data_parse(char rx_data, uint8_t uart_num);


#endif /* HAL_Y_USART_H */
