/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : C题 寻迹小车自动装卸系统 — STM32端主程序
  *
  * 功能说明:
  *   1. 上电后OLED显示卸载顺序菜单, 用户通过按键设置:
  *      - 单击: 移动光标(切换选中位置1/2/3)
  *      - 双击: 切换当前位置的物块类型(A/B/C)
  *      - 长按: 确认并启动任务
  *   2. 启动后, 接收OpenMV的 "$Car:v1,v2,v3,v4!" 指令驱动电机
  *   3. OLED实时显示运行状态(正在抓取/卸载哪个物块)
  *   4. 任务完成后OLED显示"Task Complete!"
  *
  * 注意: 本工程未使用LED指示灯、超声波、红外循迹
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* ---- BSP层头文件 ---- */
#include "hal_app_motor.h"
#include "hal_y_servo.h"
#include "hal_y_soft_i2c.h"
#include "hal_y_led.h"          /* 空壳, 无实际GPIO操作 */
#include "hal_y_timer.h"
#include "hal_y_kinematics.h"
#include "hal_y_usart.h"
#include "hal_app_uart.h"

/* ---- APP层头文件 ---- */
#include "hal_app_task.h"
#include "hal_app_oled.h"
#include "hal_app_key.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  主函数入口
  */
int main(void)
{
    /* ---- HAL库初始化 ---- */
    HAL_Init();

    /* ---- 系统时钟配置(72MHz) ---- */
    SystemClock_Config();

    /* ---- 外设初始化(CubeMX生成) ---- */
    MX_GPIO_Init();
    MX_TIM2_Init();         /* 编码器D */
    MX_TIM3_Init();         /* 编码器A */
    MX_TIM4_Init();         /* 编码器B */
    MX_TIM5_Init();         /* 编码器C */
    /* MX_TIM6_Init(); */   /* 超声波计时, 本项目不用, 如果CubeMX中还保留可以留着 */
    MX_TIM7_Init();         /* 舵机软件PWM */
    MX_TIM8_Init();         /* 电机PWM */
    MX_UART5_Init();        /* 半双工串口(调试用) */
    MX_USART1_UART_Init();  /* 调试串口 */
    MX_USART2_UART_Init();  /* 备用串口 */
    MX_USART3_UART_Init();  /* 与OpenMV通信的核心串口 */

    /* ---- BSP层初始化 ---- */
    led_init();                /* 空函数, 无实际操作 */
    app_motor_init();          /* 启动编码器 + TIM8 PWM电机驱动 */
    pwmServo_init();           /* 启动 TIM7 舵机软件PWM */
    soft_i2c_gpio_init();      /* 软件I2C初始电平(给OLED用) */
    app_uart_init();           /* 使能串口接收中断 */

    /* 机械臂运动学参数: L0=100mm底座, L1=105mm大臂, L2=88mm小臂, L3=155mm末端 */
    setup_kinematics(100.0f, 105.0f, 88.0f, 155.0f, &kinematics);

    /* ---- APP层初始化 ---- */
    App_Task_Init();           /* OLED + 按键菜单 + 任务状态机 */

    /* 延时500ms等待外设稳定(原来的LED闪烁提示去掉了) */
    HAL_Delay(500);

    /* ---- 主循环 ---- */
    while (1)
    {
        /* 任务调度主函数:
           - 菜单阶段: 处理按键, 刷新OLED
           - 运行阶段: 解析OpenMV指令, 驱动电机, 刷新OLED状态
           - 完成阶段: 停车, 显示完成 */
        App_Task_Run();
    }
}

/**
  * @brief  系统时钟配置 — 72MHz (HSE 8MHz + PLL x9)
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  错误处理函数
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* 可以在这里添加打印信息方便调试 */
}
#endif