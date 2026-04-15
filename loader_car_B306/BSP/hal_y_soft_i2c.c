/**
 * @file    hal_y_soft_i2c.c
 * @brief   软件模拟 I2C 驱动 — HAL库版本
 *
 * 移植要点：
 *   GPIO 操作从标准库 GPIO_SetBits/ResetBits/ReadInputDataBit
 *   改为 HAL_GPIO_WritePin / HAL_GPIO_ReadPin
 *   GPIO 初始化由 CubeMX 生成（配置为 Open-Drain 输出），
 *   soft_i2c_gpio_init() 仅设置初始电平
 */

#include "hal_y_soft_i2c.h"

#define I2C_TIMEOUT_TIMES  100U

/* ---- GPIO 操作宏（方便移植，修改引脚只需改头文件）---- */
#define I2C_SCL_H()  HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_L()  HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)
#define I2C_SDA_H()  HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_L()  HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)
#define I2C_SDA_READ() HAL_GPIO_ReadPin(I2C_SDA_GPIO_PORT, I2C_SDA_PIN)

static void i2c_delay(void)
{
    volatile uint8_t i = 2;
    while (--i);
}

/**
 * @brief  初始化软件 I2C 引脚
 * @note   CubeMX 已将 SCL/SDA 配置为 OD 输出，此处拉高总线
 */
void soft_i2c_gpio_init(void)
{
    I2C_SCL_H();
    I2C_SDA_H();
}

void i2c_start(void)
{
    I2C_SDA_H();
    I2C_SCL_H();
    i2c_delay();
    I2C_SDA_L();
    i2c_delay();
    I2C_SCL_L();
}

void i2c_stop(void)
{
    I2C_SDA_L();
    I2C_SCL_H();
    i2c_delay();
    I2C_SDA_H();
    i2c_delay();
}

void i2c_ack(void)
{
    I2C_SCL_L();
    I2C_SDA_L();
    i2c_delay();
    I2C_SCL_H();
    i2c_delay();
    I2C_SCL_L();
    I2C_SDA_H();
}

void i2c_nack(void)
{
    I2C_SCL_L();
    I2C_SDA_H();
    i2c_delay();
    I2C_SCL_H();
    i2c_delay();
    I2C_SCL_L();
}

/**
 * @brief  等待从机 ACK
 * @return 0=成功，1=超时
 */
uint8_t i2c_wait_ack(void)
{
    uint16_t timeout = 0;

    I2C_SCL_H();
    I2C_SDA_H();
    i2c_delay();

    while (I2C_SDA_READ() == GPIO_PIN_SET)
    {
        timeout++;
        i2c_delay();
        if (timeout > I2C_TIMEOUT_TIMES)
        {
            i2c_stop();
            return 1;
        }
    }
    I2C_SCL_L();
    return 0;
}

void i2c_write_byte(uint8_t dat)
{
    uint8_t i;
    I2C_SCL_L();
    for (i = 0; i < 8; i++)
    {
        if (dat & 0x80)
            I2C_SDA_H();
        else
            I2C_SDA_L();
        dat <<= 1;
        i2c_delay();
        I2C_SCL_H();
        i2c_delay();
        I2C_SCL_L();
        i2c_delay();
    }
}

uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    for (i = 0; i < 8; i++)
    {
        I2C_SCL_L();
        i2c_delay();
        I2C_SCL_H();
        receive <<= 1;
        if (I2C_SDA_READ() == GPIO_PIN_SET)
            receive++;
        i2c_delay();
    }
    if (!ack)
        i2c_nack();
    else
        i2c_ack();
    return receive;
}
