#ifndef HAL_Y_SOFT_I2C_H
#define HAL_Y_SOFT_I2C_H

#include "stm32f1xx_hal.h"

/* ---- I2C 引脚定义（根据实际硬件修改）---- */
#define I2C_SCL_GPIO_PORT   GPIOB
#define I2C_SCL_PIN         GPIO_PIN_8

#define I2C_SDA_GPIO_PORT   GPIOB
#define I2C_SDA_PIN         GPIO_PIN_9

void    soft_i2c_gpio_init(void);
void    i2c_start(void);
void    i2c_stop(void);
void    i2c_ack(void);
void    i2c_nack(void);
uint8_t i2c_wait_ack(void);
void    i2c_write_byte(uint8_t dat);
uint8_t i2c_read_byte(uint8_t ack);

#endif /* HAL_Y_SOFT_I2C_H */
