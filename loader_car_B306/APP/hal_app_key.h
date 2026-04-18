/**
 * @file    hal_app_key.h
 * @brief   单按键驱动头文件 — 支持单击/双击/长按
 *
 * 功能说明:
 *   只用一个GPIO按键, 通过不同操作方式实现三种事件:
 *   - 单击: 切换当前选中的位置(第1/2/3个卸载位)
 *   - 双击: 在当前位置切换物块类型(A→B→C→A循环)
 *   - 长按: 确认设置并启动任务
 *
 * 使用方法:
 *   1. CubeMX中配置按键引脚为GPIO_Input, 内部上拉
 *   2. 在main.c的while(1)中每10ms调用一次 Key_Scan()
 *   3. 调用 Key_GetEvent() 获取按键事件
 */

#ifndef HAL_APP_KEY_H
#define HAL_APP_KEY_H

#include "stm32f1xx_hal.h"

/* ---- 按键引脚定义(根据你的实际接线修改) ---- */
#define KEY_GPIO_PORT   GPIOA
#define KEY_GPIO_PIN    GPIO_PIN_0

/* ---- 按键事件枚举 ---- */
typedef enum {
    KEY_EVENT_NONE   = 0,   /* 无事件 */
    KEY_EVENT_CLICK  = 1,   /* 单击 */
    KEY_EVENT_DOUBLE = 2,   /* 双击 */
    KEY_EVENT_LONG   = 3    /* 长按(>1秒) */
} KeyEvent_t;

/* ---- 函数声明 ---- */

/**
 * @brief  按键扫描(每10ms调用一次)
 * @note   内部实现消抖和单击/双击/长按判断
 */
void Key_Scan(void);

/**
 * @brief  获取按键事件(获取后自动清除)
 * @return KeyEvent_t 按键事件类型
 */
KeyEvent_t Key_GetEvent(void);

/**
 * @brief  读取按键当前电平
 * @return 0=按下, 1=松开 (假设按下接地, 上拉输入)
 */
uint8_t Key_ReadPin(void);

#endif /* HAL_APP_KEY_H */
