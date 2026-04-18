/**
 * @file    hal_app_key.c
 * @brief   单按键驱动 — 支持单击/双击/长按
 *
 * 按键接线: 一端接GPIO(内部上拉), 另一端接GND
 * 按下=低电平, 松开=高电平
 *
 * 检测原理:
 *   1. 每10ms调用Key_Scan(), 读取按键电平
 *   2. 消抖: 连续3次(30ms)相同电平才认为有效
 *   3. 按下后开始计时:
 *      - 按下时间 > 1000ms → 长按事件(松开时触发)
 *      - 按下时间 < 500ms → 短按, 松开后等待300ms:
 *        - 300ms内再次按下松开 → 双击
 *        - 300ms内无操作 → 单击
 */

#include "hal_app_key.h"

/* ---- 私有变量 ---- */
static uint8_t    key_state = 0;        /* 状态机状态 */
static uint16_t   key_press_cnt = 0;    /* 按下持续时间计数(×10ms) */
static uint16_t   key_release_cnt = 0;  /* 松开后等待计数(×10ms) */
static uint8_t    key_click_count = 0;  /* 短按次数(用于判断单击/双击) */
static KeyEvent_t key_event = KEY_EVENT_NONE;  /* 当前事件 */
static uint8_t    key_debounce_cnt = 0; /* 消抖计数 */
static uint8_t    key_stable = 1;       /* 消抖后的稳定电平(1=松开) */

/* 长按阈值: 1000ms / 10ms = 100次 */
#define LONG_PRESS_THRESHOLD   100
/* 双击等待窗口: 300ms / 10ms = 30次 */
#define DOUBLE_CLICK_WINDOW    30

uint8_t Key_ReadPin(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_GPIO_PIN);
}

void Key_Scan(void)
{
    uint8_t current_pin = Key_ReadPin();

    /* ---- 消抖处理 ---- */
    if (current_pin != key_stable)
    {
        key_debounce_cnt++;
        if (key_debounce_cnt >= 3)  /* 连续30ms不同 → 确认变化 */
        {
            key_stable = current_pin;
            key_debounce_cnt = 0;
        }
    }
    else
    {
        key_debounce_cnt = 0;
    }

    /* ---- 状态机 ---- */
    switch (key_state)
    {
        case 0:  /* 空闲状态, 等待按下 */
            if (key_stable == 0)  /* 按下(低电平) */
            {
                key_state = 1;
                key_press_cnt = 0;
            }
            /* 检查是否有等待中的单击(双击窗口超时) */
            if (key_click_count > 0)
            {
                key_release_cnt++;
                if (key_release_cnt >= DOUBLE_CLICK_WINDOW)
                {
                    /* 超时, 确认为单击 */
                    key_event = KEY_EVENT_CLICK;
                    key_click_count = 0;
                    key_release_cnt = 0;
                }
            }
            break;

        case 1:  /* 按下中, 计时 */
            if (key_stable == 0)
            {
                key_press_cnt++;
                /* 长按判断(按下超过1秒) */
                if (key_press_cnt >= LONG_PRESS_THRESHOLD)
                {
                    key_state = 2;  /* 进入长按等待松开 */
                }
            }
            else  /* 松开了 */
            {
                if (key_press_cnt < LONG_PRESS_THRESHOLD)
                {
                    /* 短按 */
                    key_click_count++;
                    if (key_click_count >= 2)
                    {
                        /* 第二次短按 → 双击 */
                        key_event = KEY_EVENT_DOUBLE;
                        key_click_count = 0;
                        key_release_cnt = 0;
                    }
                    else
                    {
                        /* 第一次短按, 开始等待双击窗口 */
                        key_release_cnt = 0;
                    }
                }
                key_state = 0;
                key_press_cnt = 0;
            }
            break;

        case 2:  /* 长按, 等待松开 */
            if (key_stable == 1)  /* 松开了 */
            {
                key_event = KEY_EVENT_LONG;
                key_state = 0;
                key_press_cnt = 0;
                key_click_count = 0;
                key_release_cnt = 0;
            }
            break;

        default:
            key_state = 0;
            break;
    }
}

KeyEvent_t Key_GetEvent(void)
{
    KeyEvent_t evt = key_event;
    key_event = KEY_EVENT_NONE;  /* 读取后清除 */
    return evt;
}
