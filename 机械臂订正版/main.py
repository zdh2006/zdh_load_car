"""
==========================================================================
  文件名: main.py (OpenMV H7端主程序)
  功能:   C题"寻迹小车自动装卸系统" — OpenMV端入口
  说明:
    - 上电后等待STM32通过串口发送指令:
        #Order:ABC!  设置卸载顺序
        #Go!         启动任务
    - 主循环中不断调用 trackTask.run() 执行任务
    - cx/cy/cz 是机械臂抓取偏移量微调参数:
        cx: 抓取位置偏右了就减小, 偏左了就增加
        cy: 抓取位置偏前了就减小, 偏后了就增加
        cz: 抓取位置偏高了就减小, 偏低了就增加
==========================================================================
"""

from pyb import UART, Pin, Timer
import time
import trackTask


# ======================== 微调参数(根据实际调整) ========================
# 如果机械臂抓取物块总是偏, 在这里调整:
cx = 0   # 左右偏移补偿(mm)
cy = 0   # 前后偏移补偿(mm)
cz = 0   # 高度偏移补偿(mm)

# ======================== 创建任务对象 ========================
task = trackTask.TrackTask()

if __name__ == "__main__":
    # 初始化(摄像头、补光灯、机械臂巡线姿态)
    task.init()

    print("=== OpenMV H7 Ready ===")
    print("Waiting for STM32 commands...")
    print("  #Order:ABC!  - Set unload order")
    print("  #Go!         - Start task")

    # 主循环
    while True:
        task.run(cx, cy, cz)
