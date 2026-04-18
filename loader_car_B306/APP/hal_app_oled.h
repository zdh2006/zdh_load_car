/**
 * @file    hal_app_oled.h
 * @brief   OLED显示驱动头文件 — 使用SSD1306 128x64 I2C OLED
 *
 * 功能:
 *   - 基于软件I2C驱动SSD1306
 *   - 提供清屏、显示字符串、显示一行文字等接口
 *   - 用于按键菜单界面显示卸载顺序
 */

#ifndef HAL_APP_OLED_H
#define HAL_APP_OLED_H

#include "stm32f1xx_hal.h"

/* SSD1306 I2C地址(7位地址左移1位) */
#define OLED_I2C_ADDR  0x78

/* 屏幕尺寸 */
#define OLED_WIDTH   128
#define OLED_HEIGHT  64

/* ---- 初始化 ---- */
void OLED_Init(void);

/* ---- 基本操作 ---- */
void OLED_Clear(void);
void OLED_Display_On(void);
void OLED_Display_Off(void);

/* ---- 显示函数 ---- */
/* 在指定行(0~7)列(0~127)显示一个字符串(6x8字体) */
void OLED_ShowString(uint8_t row, uint8_t col, const char *str);

/* 清除指定行 */
void OLED_ClearRow(uint8_t row);

/* 刷新显示缓冲区到屏幕 */
void OLED_Refresh(void);

/* 设置光标位置 */
void OLED_SetCursor(uint8_t row, uint8_t col);

/* 写一个字符(6x8) */
void OLED_ShowChar(uint8_t row, uint8_t col, char ch);

#endif /* HAL_APP_OLED_H */
