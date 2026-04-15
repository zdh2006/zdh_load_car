/**
 * @file    hal_y_kinematics.c
 * @brief   4自由度机械臂逆运动学解算 — HAL库版本
 *
 * 纯数学运算，与标准库/HAL库无关，代码逻辑与原工程完全一致。
 * 仅头文件引用改为 HAL 版本。
 *
 * 使用方法：
 *   1. 调用 setup_kinematics() 设置机械臂连杆长度（单位mm）
 *   2. 调用 kinematics_analysis() 输入目标坐标，解算舵机角度和PWM值
 *   3. 检查返回值，0=成功，非0=超出工作空间或奇异点
 *
 * 本工程初始化参数：
 *   setup_kinematics(100, 105, 88, 155, &kinematics)
 *   L0=底座高100mm, L1=大臂105mm, L2=小臂88mm, L3=末端155mm
 */

#include "hal_y_kinematics.h"
#include <math.h>

#define PI  3.1415926f

kinematics_t kinematics;

/**
 * @brief  设置机械臂连杆长度
 * @param  L0  底座高度 (mm)
 * @param  L1  大臂长度 (mm)
 * @param  L2  小臂长度 (mm)
 * @param  L3  末端执行器长度 (mm)
 * @param  k   运动学结构体指针
 * @note   内部存储时×10，提高浮点精度
 */
void setup_kinematics(float L0, float L1, float L2, float L3, kinematics_t *k)
{
    k->L0 = L0 * 10.0f;
    k->L1 = L1 * 10.0f;
    k->L2 = L2 * 10.0f;
    k->L3 = L3 * 10.0f;
}

/**
 * @brief  逆运动学解算
 * @param  x      目标 X 坐标 (cm，水平平面内)
 * @param  y      目标 Y 坐标 (cm，距底座水平距离)
 * @param  z      目标 Z 坐标 (cm，距地面高度)
 * @param  Alpha  末端执行器与水平面夹角 (°)，推荐 -25 ~ -65
 * @param  k      运动学结构体指针
 * @return 0=成功；1-7=不同类型的工作空间超限错误码
 *
 * 解算结果存入：
 *   k->servo_angle[0] = 底座旋转角  (°)
 *   k->servo_angle[1] = 大臂角       (°)
 *   k->servo_angle[2] = 小臂角       (°)
 *   k->servo_angle[3] = 腕部角       (°)
 *   k->servo_pwm[0~3] = 对应 PWM 值 (500~2500 us)
 */
int kinematics_analysis(float x, float y, float z, float Alpha, kinematics_t *k)
{
    float theta3, theta4, theta5, theta6;
    float l0, l1, l2, l3;
    float aaa, bbb, ccc, zf_flag;

    /* 坐标×10（配合连杆参数的×10存储）*/
    x *= 10.0f;
    y *= 10.0f;
    z *= 10.0f;

    l0 = k->L0;
    l1 = k->L1;
    l2 = k->L2;
    l3 = k->L3;

    /* 底座旋转角（绕Z轴） */
    theta6 = (x == 0.0f) ? 0.0f : atanf(x / y) * 270.0f / PI;

    /* 投影到平面内求解 */
    y = sqrtf(x * x + y * y);
    y = y - l3 * cosf(Alpha * PI / 180.0f);
    z = z - l0 - l3 * sinf(Alpha * PI / 180.0f);

    if (z < -l0) return 1;
    if (sqrtf(y * y + z * z) > (l1 + l2)) return 2;

    /* 求大臂角 theta5 */
    ccc = acosf(y / sqrtf(y * y + z * z));
    bbb = (y * y + z * z + l1 * l1 - l2 * l2) / (2.0f * l1 * sqrtf(y * y + z * z));
    if (bbb > 1.0f || bbb < -1.0f) return 5;
    zf_flag = (z < 0.0f) ? -1.0f : 1.0f;
    theta5 = ccc * zf_flag + acosf(bbb);
    theta5 = theta5 * 180.0f / PI;
    if (theta5 > 180.0f || theta5 < 0.0f) return 6;

    /* 求小臂角 theta4 */
    aaa = -(y * y + z * z - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    if (aaa > 1.0f || aaa < -1.0f) return 3;
    theta4 = acosf(aaa);
    theta4 = 180.0f - theta4 * 180.0f / PI;
    if (theta4 > 135.0f || theta4 < -135.0f) return 4;

    /* 腕部角 theta3（保持末端与水平面夹角） */
    theta3 = Alpha - theta5 + theta4;
    if (theta3 > 90.0f || theta3 < -90.0f) return 7;

    /* 存储结果 */
    k->servo_angle[0] = theta6;
    k->servo_angle[1] = theta5 - 90.0f;
    k->servo_angle[2] = theta4;
    k->servo_angle[3] = theta3;

    /* 换算 PWM（中位=1500us，±270°对应±2000us） */
    k->servo_pwm[0] = (int)(1500.0f - 2000.0f * k->servo_angle[0] / 270.0f);
    k->servo_pwm[1] = (int)(1500.0f + 2000.0f * k->servo_angle[1] / 270.0f);
    k->servo_pwm[2] = (int)(1500.0f + 2000.0f * k->servo_angle[2] / 270.0f);
    k->servo_pwm[3] = (int)(1500.0f - 2000.0f * k->servo_angle[3] / 270.0f);

    return 0;
}
