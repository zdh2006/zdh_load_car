import sensor, time, math
from pyb import Pin,Timer,UART
import kinematics, robotMoveCmd

class CarColorTraceGrasp():
    red_threshold = (0, 100, 20, 127, 0, 127)
    blue_threshold = (0, 100, -128, 127, -128, -15)
    green_threshold = (0, 100, -128, -28, 0, 70)
    yellow_threshold = (57, 100, -33, 70, 48, 127)
    track_color_threshold = blue_threshold#追踪的颜色

    uart = UART(3,115200)   #设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1 )

    tim = Timer(4, freq=1000) # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=0)

    #物块中心点
    mid_block_cx=80
    mid_block_cy=60

    # 机械臂移动位置
    move_x = 0
    move_y = 160
    move_z = 100

    adjust_position = 0  # 用来判断小车状态
    move_status = 0  # 机械臂移动的方式
    mid_block_cnt = 0  # 用来记录机械臂已对准物块计数，防止误差

    # 机器人运动
    robot_move_cmd = robotMoveCmd.RobotMoveCmd()
    kinematic = kinematics.Kinematics()

    def init(self):
        sensor.reset() #初始化摄像头
        sensor.set_pixformat(sensor.RGB565) #图像格式为 RGB565 灰度 GRAYSCALE
        sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
        sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
        sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
        sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

        self.led_dac.pulse_width_percent(100)

        # 机械臂移动位置
        self.move_x = 0
        self.move_y = 160
        self.move_z = 70

        self.adjust_position = 0  # 用来判断小车旋转状态
        self.move_status = 0  # 机械臂移动的方式
        self.mid_block_cnt = 0  # 用来记录机械臂已对准物块计数，防止误差

        self.kinematic.kinematics_move(self.move_x, self.move_y, self.move_z, 1000)
        time.sleep_ms(1000)

    def run(self,cx=0,cy=0,cz=0):#运行功能
        '''
            3个变量控制机械臂抓取色块时的偏移量,如果机械臂抓取色块失败则调整变量
            cx: 偏右减小, 偏左增加
            cy: 偏前减小，偏后增加
            cz: 偏高减小，偏低增加
        '''

        #物块中心点
        block_cx=self.mid_block_cx
        block_cy=self.mid_block_cy
        # 获取图像
        img = sensor.snapshot()

        #***************进行色块检测********************
        blobs = img.find_blobs([self.track_color_threshold],x_stride=15, y_stride=15, pixels_threshold=25 )
        max_size = 0
        max_blob = 0
        for blob in blobs:#寻找最大
            if blob[2] * blob[3] > max_size:
                max_blob = blob
                max_size = blob[2] * blob[3]

        if blobs:#色块
            img.draw_rectangle((max_blob[0],max_blob[1],max_blob[2],max_blob[3]),color=(255,255,255))
            img.draw_cross(max_blob[5], max_blob[6],size=2,color=(255,0,0))
            img.draw_string(max_blob[0], (max_blob[1]-10), "color", color=(255,0,0))
            #print("中心X坐标",max_blob[5],"中心Y坐标",max_blob[6],"识别颜色类型","红色")
            block_cx=max_blob[5]
            block_cy=max_blob[6]

            if self.move_status == 0:  # 追踪颜色
                # ************************运动机械臂**********************************
                if abs(block_cx - self.mid_block_cx) > 5:
                    if block_cx > self.mid_block_cx:
                        self.move_x += 1
                    else:
                        self.move_x -= 1
                if abs(block_cy - self.mid_block_cy) > 5:
                    if block_cy > self.mid_block_cy and self.move_z > 1:
                        self.move_z -= 2
                    else:
                        self.move_z += 2
                self.kinematic.kinematics_move(self.move_x, self.move_y, self.move_z, 0)
                time.sleep_ms(10)

                if abs(self.move_z - 70) > 10 or abs(self.move_x - 0) > 10:  # 寻找到物块
                    # ************************运动机器人*********************************
                    if self.move_x > 10:  # 右转
                        self.mid_block_cnt = 0
                        if self.adjust_position != 3:
                            self.adjust_position = 3
                            self.robot_move_cmd.car_move(0.05, -0.05, 0.05, -0.05)
                    elif self.move_x < -10:  # 左转
                        self.mid_block_cnt = 0
                        if self.adjust_position != 2:
                            self.adjust_position = 2
                            self.robot_move_cmd.car_move(-0.05, 0.05, -0.05, 0.05)
                    elif self.move_z - 70 > 10:  # 前进
                        self.mid_block_cnt = 0
                        if self.adjust_position != 1:
                            self.adjust_position = 1
                            self.robot_move_cmd.car_move(0.05, 0.05, 0.05, 0.05)
                    elif self.move_z - 70 < -10:  # 后退
                        self.mid_block_cnt = 0
                        if self.adjust_position != 4:
                            self.adjust_position = 4
                            self.robot_move_cmd.car_move(-0.05, -0.05, -0.05, -0.05)
                else:  # 调整完毕，停止
                    if self.adjust_position != 5:
                        self.adjust_position = 5
                        self.robot_move_cmd.car_move(0, 0, 0, 0)
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 10:  # 计数10次对准物块，防止误差
                        self.move_status = 1
                        self.mid_block_cnt=0
            elif self.move_status == 1:  # 机械臂寻找物块
                if(abs(block_cx-self.mid_block_cx)>3):
                    if block_cx > self.mid_block_cx:
                        self.move_x+=0.2
                    else:
                        self.move_x-=0.2
                if(abs(block_cy-self.mid_block_cy)>3):
                    if block_cy > self.mid_block_cy and self.move_y>80:
                        self.move_y-=0.3
                    else:
                        self.move_y+=0.3
                if abs(block_cy-self.mid_block_cy)<=3 and abs(block_cx-self.mid_block_cx)<=3: #寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt>10:#计数10次对准物块，防止误差
                        self.mid_block_cnt=0
                        self.move_status=2
                else:
                    self.mid_block_cnt=0
                    self.kinematic.kinematics_move(self.move_x,self.move_y,70,10)
                time.sleep_ms(10)

            elif self.move_status == 2:  # 机械臂抓取物块
                self.move_status = 0
                l = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = (l + 99+cy) * cos+cx
                self.move_y = (l + 99+cy) * sin
                time.sleep_ms(100)
                self.kinematic.send_str("#005P1000T1000!")  # 机械爪张开
                time.sleep_ms(100)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)  # 移动机械臂到物块上方
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, -75+cz, 1000)
                time.sleep_ms(1200)
                self.kinematic.send_str("#005P1700T1000!")  # 机械爪闭合
                time.sleep_ms(1200)
                self.move_x = 0  # 机械臂归位
                self.move_y = 160
                self.move_z = 70
                self.kinematic.kinematics_move(self.move_x, self.move_y, self.move_z, 1000)
                time.sleep_ms(1200)
        else:
            if self.adjust_position != 5:
                self.adjust_position = 5
                self.robot_move_cmd.car_move(0, 0, 0, 0)



if __name__ == "__main__":
    app=CarColorTraceGrasp()
    app.init()#初始化

    while(1):
        app.run(0,0,0)#运行功能








