#ifndef HAL_APP_UART_H
#define HAL_APP_UART_H

#include "stm32f1xx_hal.h"
#include "hal_y_usart.h"
#include "usart.h"   /* CubeMX 生成：huart1/2/3/5 */

/* 这些函数在 resource.c / y_global.c 中定义，根据原工程保留 */
// extern void parse_cmd(char *buf);
// extern void parse_action(char *buf);
// extern void save_action(char *buf);

void app_uart_init(void);
// void app_uart_run(void);

#endif /* HAL_APP_UART_H */
