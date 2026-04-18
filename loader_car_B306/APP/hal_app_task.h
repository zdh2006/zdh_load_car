/**
 * @file    hal_app_task.h
 * @brief   STM32端总任务调度头文件
 *
 * 功能:
 *   1. 按键菜单: 通过单击/双击/长按设置卸载顺序
 *   2. OLED显示: 实时显示当前设置和运行状态
 *   3. 串口通信: 接收OpenMV的控制指令, 驱动电机
 *   4. 串口通信: 向OpenMV发送卸载顺序和启动指令
 */

#ifndef HAL_APP_TASK_H
#define HAL_APP_TASK_H

#include "stm32f1xx_hal.h"

/* ---- 任务状态枚举 ---- */
typedef enum {
    TASK_STATE_MENU   = 0,   /* 菜单设置状态(按键操作OLED) */
    TASK_STATE_RUNNING = 1,  /* 运行中(接收OpenMV指令控制电机) */
    TASK_STATE_DONE   = 2    /* 任务完成 */
} TaskState_t;

/* ---- 初始化 ---- */
void App_Task_Init(void);

/* ---- 主循环调用(放在while(1)中) ---- */
void App_Task_Run(void);

/* ---- 串口数据处理(在串口中断中调用) ---- */
void App_Task_UartProcess(void);

#endif /* HAL_APP_TASK_H */
