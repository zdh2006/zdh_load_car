from pyb import UART, Pin, Timer
import time, pyb
import apriltagPalletizer,apriltagSort
import armColorTrace,carColorTrace,carColorTraceGrasp
import lineSortMap1,lineSortMap2,lineMap3
import faceTrack

apriltag_Palletizer = apriltagPalletizer.ApriltagPalletizer()
apriltag_Sort = apriltagSort.ApriltagSort()
arm_Color_Trace = armColorTrace.ArmColorTrace()
car_Color_Trace = carColorTrace.CarColorTrace()
car_Color_Trace_Grasp = carColorTraceGrasp.CarColorTraceGrasp()
line_Sort_Map1 = lineSortMap1.LineSortMap1()
line_Sort_Map2 = lineSortMap2.LineSortMap2()
line_Map3 = lineMap3.LineMap3()
face_Track = faceTrack.FaceTrack()

led = pyb.LED(3)

uart = UART(3, 115200)  # 设置串口波特率，与stm32一致
uart.init(115200, bits=8, parity=None, stop=1)

tim = Timer(4, freq=1000)  # Frequency in Hz
led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=50)
led_dac.pulse_width_percent(0)

uart.write("$KMS:0,160,50,1000!\n")

run_app_status = 0

uart.write("#openmv reset!")

'''
    3个变量控制机械臂抓取色块时的偏移量,如果机械臂抓取色块失败则调整变量
    cx: 偏右减小, 偏左增加
    cy: 偏前减小，偏后增加
    cz: 偏高减小，偏低增加
'''
cx=0
cy=0
cz=0

if __name__ == "__main__":
    while (True):
        if uart.any():  # 接收指令
            try:  # 用来判断串口数据异常
                string = uart.read()
                if string:
                    string = string.decode()
                    # print(string, run_app_status,string.find("#start_led!"))
                    if string.find("#StartLed!") >= 0:  # 开灯指令
                        led_dac.pulse_width_percent(100)
                    elif string.find("#StopLed!") >= 0:  # 关灯指令
                        led_dac.pulse_width_percent(0)
                    elif string.find("#RunStop!") >= 0:  # 停止所有运行并复位
                        run_app_status = 0
                        for j in range(3):
                            uart.write("$MV0!")
                            time.sleep_ms(100)
                        uart.write("$Car:0,0,0,0!\n")  # 发送控制小车行动的指令
                        uart.write("$KMS:0,160,50,1000!\n")
                        led_dac.pulse_width_percent(0)
                    elif string.find("#CarColorTrace!") >= 0:  # 小车颜色追踪
                        run_app_status = 1
                        for j in range(3):
                            uart.write("$MV1!")
                            time.sleep_ms(100)
                        car_Color_Trace.init()
                    elif string.find("#Map3LineWalk!") >= 0:  # 巡线地图3
                        run_app_status = 2
                        for j in range(3):
                            uart.write("$MV2!")
                            time.sleep_ms(100)
                        line_Map3.init()
                    elif string.find("#ArmColorTrace!") >= 0:  # 机械臂颜色追踪
                        run_app_status = 3
                        for j in range(3):
                            uart.write("$MV3!")
                            time.sleep_ms(100)
                        arm_Color_Trace.init()
                    elif string.find("#AprilTagSort!") >= 0:  # 二维码标签分拣
                        run_app_status = 4
                        for j in range(3):
                            uart.write("$MV4!")
                            time.sleep_ms(100)
                        apriltag_Sort.init()
                    elif string.find("#AprilTagStack!") >= 0:  # 二维码标签码垛
                        run_app_status = 5
                        for j in range(3):
                            uart.write("$MV5!")
                            time.sleep_ms(100)
                        apriltag_Palletizer.init()
                    elif string.find("#TraceGrasp!") >= 0:  # 颜色追踪抓取
                        run_app_status = 6
                        for j in range(3):
                            uart.write("$MV6!")
                            time.sleep_ms(100)
                        car_Color_Trace_Grasp.init()
                    elif string.find("#Map1LineWalk!") >= 0:  # 地图1巡线
                        run_app_status = 7
                        for j in range(3):
                            uart.write("$MV7!")
                            time.sleep_ms(100)
                        line_Sort_Map1.init()
                    elif string.find("#Map2LineWalk!") >= 0:  # 地图2巡线
                        run_app_status = 8
                        for j in range(3):
                            uart.write("$MV8!")
                            time.sleep_ms(100)
                        line_Sort_Map2.init()
                    elif string.find("#FaceTrack!") >= 0:  # 人脸识别
                        run_app_status = 9
                        for j in range(3):
                            uart.write("$MV9!")
                            time.sleep_ms(100)
                        face_Track.init()
            except Exception as e:  # 串口数据异常进入
                print('Error:', e)

        if run_app_status == 1:
            car_Color_Trace.run()  # 运行小车颜色追踪
        elif run_app_status == 2:
            line_Map3.run()  # 运行地图3巡线功能
        elif run_app_status == 3:
            arm_Color_Trace.run()  # 运行平台颜色追踪
        elif run_app_status == 4:
            apriltag_Sort.run(cx,cy,cz)  # 运行二维码标签分拣
        elif run_app_status == 5:
            apriltag_Palletizer.run(cx,cy,cz)  # 运行二维码标签码垛
        elif run_app_status == 6:
            car_Color_Trace_Grasp.run(cx,cy,cz)  # 运行颜色追踪抓取功能，需要换上黑金大爪子
        elif run_app_status == 7:
            line_Sort_Map1.run(cx,cy,cz)  # 运行地图1巡线功能
        elif run_app_status == 8:
            line_Sort_Map2.run(cx,cy,cz)  # 运行地图2巡线功能
        elif run_app_status == 9:
            face_Track.run()  # 运行人脸追踪功能
