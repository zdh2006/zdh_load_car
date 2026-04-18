/**
 * @file    hal_app_task.c
 * @brief   STM32端总任务调度 — C题核心
 *
 * ===================== 整体架构 =====================
 *
 * 【菜单阶段】(TASK_STATE_MENU)
 *   OLED显示卸载顺序, 用户通过按键操作:
 *     单击 → 移动光标到下一个位置(位置1→2→3→1循环)
 *     双击 → 当前位置的物块类型切换(A→B→C→A循环)
 *     长按 → 确认设置, 通过串口发送顺序给OpenMV, 启动任务
 *
 *   OLED显示效果示例:
 *   ┌──────────────────┐
 *   │  Unload Setting  │  ← 第0行: 标题
 *   │                  │
 *   │  Pos1: [A]       │  ← 第2行: 位置1 (带方括号=当前选中)
 *   │  Pos2:  B        │  ← 第3行: 位置2
 *   │  Pos3:  C        │  ← 第4行: 位置3
 *   │                  │
 *   │ Click:Move       │  ← 第6行: 操作提示
 *   │ Dbl:Change Long:Go│ ← 第7行: 操作提示
 *   └──────────────────┘
 *
 * 【运行阶段】(TASK_STATE_RUNNING)
 *   接收OpenMV发来的 "$Car:v1,v2,v3,v4!" 指令, 驱动电机
 *   接收OpenMV发来的 "$Grab:A!" "$Unload:B!" "$Done!" 等状态信息, OLED显示
 *
 * 【完成阶段】(TASK_STATE_DONE)
 *   OLED显示"Task Complete!"
 */

#include "hal_app_task.h"
#include "usart.h"
#include "tim.h"         // 定时器句柄 htim2/3/4/5/6/7/8
#include "gpio.h"        // GPIO 相关
#include "hal_app_key.h"
#include "hal_app_oled.h"
#include "hal_app_motor.h"
#include "hal_y_usart.h"
#include "hal_y_delay.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ================ 私有变量 ================ */

/* 任务总状态 */
static TaskState_t task_state = TASK_STATE_MENU;

/* 卸载顺序: unload_order[0]='A', [1]='B', [2]='C' 表示先卸A再卸B再卸C */
static char unload_order[3] = {'A', 'B', 'C'};

/* 菜单光标位置: 0=位置1, 1=位置2, 2=位置3 */
static uint8_t menu_cursor = 0;

/* OLED刷新标志(有变化才刷新, 避免频繁刷屏闪烁) */
static uint8_t oled_need_refresh = 1;

/* 运行时OLED显示的状态文字 */
static char run_status_line1[22] = "Running...";
static char run_status_line2[22] = "";
static char run_status_line3[22] = "";

/* 10ms定时计数(用于按键扫描) */
static uint32_t last_key_tick = 0;

/* ================ 私有函数声明 ================ */
static void menu_display(void);
static void menu_handle_key(KeyEvent_t evt);
static void running_display(void);
static void parse_openmv_cmd(const char *buf);
static char next_block_char(char current);

/* ================ 初始化 ================ */

void App_Task_Init(void)
{
    /* 初始化OLED */
    OLED_Init();

    /* 初始化按键(GPIO已由CubeMX配置) */
    /* 无需额外初始化 */

    /* 默认卸载顺序A→B→C */
    unload_order[0] = 'A';
    unload_order[1] = 'B';
    unload_order[2] = 'C';
    menu_cursor = 0;
    task_state = TASK_STATE_MENU;
    oled_need_refresh = 1;

    /* 使能串口3接收中断(与OpenMV通信的串口) */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
    /* 使能串口1接收中断(调试用, 可选) */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    last_key_tick = HAL_GetTick();
}

/* ================ 主循环 ================ */

void App_Task_Run(void)
{
    /* ---- 每10ms扫描一次按键 ---- */
    if (HAL_GetTick() - last_key_tick >= 10)
    {
        last_key_tick = HAL_GetTick();
        Key_Scan();
    }

    /* ---- 根据任务状态执行不同逻辑 ---- */
    switch (task_state)
    {
        /* ======== 菜单设置阶段 ======== */
        case TASK_STATE_MENU:
        {
            /* 获取按键事件 */
            KeyEvent_t evt = Key_GetEvent();
            if (evt != KEY_EVENT_NONE)
            {
                menu_handle_key(evt);
            }

            /* 刷新OLED显示 */
            if (oled_need_refresh)
            {
                menu_display();
                oled_need_refresh = 0;
            }
            break;
        }

        /* ======== 运行阶段 ======== */
        case TASK_STATE_RUNNING:
        {
            /* 处理OpenMV发来的串口指令 */
            if (uart_get_ok)
            {
                parse_openmv_cmd(uart_receive_buf);
                uart_get_ok = 0;
                uart_mode = 0;
            }

            /* 刷新OLED显示 */
            if (oled_need_refresh)
            {
                running_display();
                oled_need_refresh = 0;
            }
            break;
        }

        /* ======== 完成阶段 ======== */
        case TASK_STATE_DONE:
        {
            if (oled_need_refresh)
            {
                OLED_Clear();
                OLED_ShowString(2, 10, "Task Complete!");
                OLED_ShowString(4, 10, "All Done :)");
                oled_need_refresh = 0;
            }
            /* 确保电机停止 */
            motor_speed_set(0, 0, 0, 0);
            break;
        }
    }
}

/* ================ 菜单显示 ================ */

static void menu_display(void)
{
    char line[22];

    OLED_Clear();

    /* 第0行: 标题 */
    OLED_ShowString(0, 10, "Unload Setting");

    /* 第2~4行: 三个卸载位置 */
    for (uint8_t i = 0; i < 3; i++)
    {
        if (i == menu_cursor)
        {
            /* 当前选中位置: 用方括号和箭头标记 */
            snprintf(line, sizeof(line), "->Pos%d: [%c]", i + 1, unload_order[i]);
        }
        else
        {
            /* 非选中位置 */
            snprintf(line, sizeof(line), "  Pos%d:  %c ", i + 1, unload_order[i]);
        }
        OLED_ShowString(2 + i, 0, line);
    }

    /* 第6行: 操作提示 */
    OLED_ShowString(6, 0, "Click:Move Pos");
    OLED_ShowString(7, 0, "Dbl:Chg Long:Go!");
}

/* ================ 菜单按键处理 ================ */

static void menu_handle_key(KeyEvent_t evt)
{
    switch (evt)
    {
        case KEY_EVENT_CLICK:
            /* 单击: 光标移到下一个位置(0→1→2→0循环) */
            menu_cursor++;
            if (menu_cursor >= 3) menu_cursor = 0;
            oled_need_refresh = 1;
            break;

        case KEY_EVENT_DOUBLE:
            /* 双击: 当前位置的物块类型切换(A→B→C→A)
               同时检查重复: 如果切换后与其他位置重复, 自动跳过 */
        {
            char new_block = next_block_char(unload_order[menu_cursor]);
            /* 检查重复, 最多尝试3次 */
            for (uint8_t try = 0; try < 3; try++)
            {
                uint8_t duplicate = 0;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (j != menu_cursor && unload_order[j] == new_block)
                    {
                        duplicate = 1;
                        break;
                    }
                }
                if (!duplicate) break;
                new_block = next_block_char(new_block);
            }
            unload_order[menu_cursor] = new_block;
            oled_need_refresh = 1;
            break;
        }

        case KEY_EVENT_LONG:
            /* 长按: 确认设置, 发送卸载顺序给OpenMV, 启动任务 */
        {
            char order_cmd[20];

            /* 显示"正在启动..." */
            OLED_Clear();
            OLED_ShowString(2, 10, "Starting...");
            snprintf(order_cmd, sizeof(order_cmd), "Order: %c%c%c",
                     unload_order[0], unload_order[1], unload_order[2]);
            OLED_ShowString(4, 10, order_cmd);

            /* 通过串口3发送卸载顺序给OpenMV */
            /* 格式: #Order:ABC!  */
            snprintf(order_cmd, sizeof(order_cmd), "#Order:%c%c%c!",
                     unload_order[0], unload_order[1], unload_order[2]);

            /* 多发几次确保OpenMV收到 */
            for (uint8_t i = 0; i < 5; i++)
            {
                uart3_send_str(order_cmd);
                HAL_Delay(50);
            }

            HAL_Delay(500);

            /* 发送启动指令 */
            for (uint8_t i = 0; i < 5; i++)
            {
                uart3_send_str("#Go!");
                HAL_Delay(50);
            }

            /* 切换到运行状态 */
            snprintf(run_status_line1, sizeof(run_status_line1),
                     "Order:%c->%c->%c",
                     unload_order[0], unload_order[1], unload_order[2]);
            snprintf(run_status_line2, sizeof(run_status_line2), "Running...");
            snprintf(run_status_line3, sizeof(run_status_line3), "");

            task_state = TASK_STATE_RUNNING;
            oled_need_refresh = 1;
            break;
        }

        default:
            break;
    }
}

/* ================ 运行时OLED显示 ================ */

static void running_display(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "=== RUNNING ===");
    OLED_ShowString(2, 0, run_status_line1);
    OLED_ShowString(4, 0, run_status_line2);
    OLED_ShowString(6, 0, run_status_line3);
}

/* ================ 解析OpenMV发来的指令 ================ */

/**
 * @brief  解析OpenMV通过串口3发来的指令
 *
 * 支持的指令格式:
 *   "$Car:v1,v2,v3,v4!"   — 控制四轮电机速度
 *   "$Grab:A!"             — 正在抓取物块A(OLED显示)
 *   "$Unload:B!"           — 正在卸载物块B(OLED显示)
 *   "$Done!"               — 任务完成
 */
static void parse_openmv_cmd(const char *buf)
{
    /* ---- 解析电机控制指令: $Car:v1,v2,v3,v4! ---- */
    if (strstr(buf, "$Car:") != NULL)
    {
        float v1 = 0, v2 = 0, v3 = 0, v4 = 0;
        const char *p = strstr(buf, "$Car:") + 5;

        /* 手动解析4个浮点数(用逗号分隔) */
        v1 = (float)atof(p);
        p = strchr(p, ',');
        if (p) { p++; v2 = (float)atof(p); }
        p = strchr(p, ',');
        if (p) { p++; v3 = (float)atof(p); }
        p = strchr(p, ',');
        if (p) { p++; v4 = (float)atof(p); }

        /* 设置电机速度 */
        motor_speed_set(v1, v2, v3, v4);

        /* 回复确认(OpenMV的RobotMoveCmd需要收到"cmdOk") */
        uart3_send_str("cmdOk");
    }

    /* ---- 解析抓取信息: $Grab:A! ---- */
    else if (strstr(buf, "$Grab:") != NULL)
    {
        char block_name = *(strstr(buf, "$Grab:") + 6);
        snprintf(run_status_line2, sizeof(run_status_line2),
                 "Grabbed: %c", block_name);
        oled_need_refresh = 1;

        uart3_send_str("cmdOk");
    }

    /* ---- 解析卸载信息: $Unload:B! ---- */
    else if (strstr(buf, "$Unload:") != NULL)
    {
        char block_name = *(strstr(buf, "$Unload:") + 8);
        snprintf(run_status_line2, sizeof(run_status_line2),
                 "Unloading: %c", block_name);
        oled_need_refresh = 1;

        uart3_send_str("cmdOk");
    }

    /* ---- 解析任务完成: $Done! ---- */
    else if (strstr(buf, "$Done!") != NULL)
    {
        task_state = TASK_STATE_DONE;
        oled_need_refresh = 1;

        uart3_send_str("cmdOk");
    }

    /* ---- 运动学指令透传: $KMS:x,y,z,t! ---- */
    else if (strstr(buf, "$KMS:") != NULL)
    {
        /* 暂时不需要处理, 保留接口 */
        uart3_send_str("cmdOk");
    }
}

/* ================ 辅助函数 ================ */

/**
 * @brief  获取下一个物块字符(A→B→C→A循环)
 */
static char next_block_char(char current)
{
    switch (current)
    {
        case 'A': return 'B';
        case 'B': return 'C';
        case 'C': return 'A';
        default:  return 'A';
    }
}

/* ================ 串口数据处理(在中断中调用) ================ */

void App_Task_UartProcess(void)
{
    /* 此函数由串口中断调用, 将接收到的字节送入协议解析器
       实际的字节级处理在 stm32f1xx_it.c 的 USARTx_IRQHandler 中完成
       这里只是一个兼容接口 */
}
