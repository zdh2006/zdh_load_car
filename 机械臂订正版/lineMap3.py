import sensor, time, math
from pyb import Pin,Timer,UART
import kinematics, robotMoveCmd

class LineMap3():
    # 设置阈值，如果是黑线，GRAYSCALE_THRESHOLD = [(0, 64)]；
    # 如果是白线，GRAYSCALE_THRESHOLD = [(128，255)]
    GRAYSCALE_THRESHOLD = [(0, 64)]

    # 每个roi为(x, y, w, h)，线检测算法将尝试找到每个roi中最大的blob的质心。
    # 然后用不同的权重对质心的x位置求平均值，其中最大的权重分配给靠近图像底部的roi，
    # 较小的权重分配给下一个roi，以此类推。
    ROIS = [  # [ROI, weight]
        (0, 100, 200, 20, 0.5, 1),  # 你需要为你的应用程序调整权重
        (0, 75, 200, 20, 0.4, 2),
        (0, 50, 200, 20, 0.3, 3),  # 取决于你的机器人是如何设置的。
        (0, 25, 200, 20, 0.2, 4),
        (0, 000, 200, 20, 0.1, 5)
    ]
    # roi代表三个取样区域，（x,y,w,h,weight）,代表左上顶点（x,y）宽高分别为w和h的矩形，
    # weight为当前矩形的权值。注意本例程采用的QQVGA图像大小为160x120，roi即把图像横分成三个矩形。
    # 三个矩形的阈值要根据实际情况进行调整，离机器人视野最近的矩形权值要最大，
    # 如上图的最下方的矩形，即(0, 100, 200, 20, 0.7,1)（最后一个值1是用来记录的）


    red_threshold = (0, 100, 20, 127, 0, 127)
    blue_threshold = (0, 100, -128, 127, -128, -15)
    green_threshold = (0, 100, -128, -28, 0, 70)
    yellow_threshold = (57, 100, -33, 70, 48, 127)
    track_color_threshold = blue_threshold#追踪的颜色

    uart = UART(3,115200)   #设置串口波特率，与stm32一致
    uart.init(115200, bits=8, parity=None, stop=1 )

    tim = Timer(4, freq=1000) # Frequency in Hz
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=0)


    # 机器人运动
    robot_move_cmd = robotMoveCmd.RobotMoveCmd()
    kinematic = kinematics.Kinematics()

    def init(self):
        sensor.reset() #初始化摄像头
        sensor.set_pixformat(sensor.GRAYSCALE) #图像格式为 RGB565 灰度 GRAYSCALE
        sensor.set_framesize(sensor.QQVGA) #QQVGA: 160x120
        sensor.skip_frames(n=2000) #在更改设置后，跳过n张照片，等待感光元件变稳定
        sensor.set_auto_gain(True) #使用颜色识别时需要关闭自动自动增益
        sensor.set_auto_whitebal(True)#使用颜色识别时需要关闭自动自动白平衡

        self.led_dac.pulse_width_percent(100)

        # 机械臂移动位置
        self.move_x = 0
        self.move_y = 160

        self.crossing_cnt = 0  # MAP3路口计数
        self.speed_motor1 = 0
        self.speed_motor2 = 0
        self.speed_motor3 = 0
        self.speed_motor4 = 0
        self.crossing_flag = 0  # 标记路口情况计数，判断是否经过一个路口

        self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
        time.sleep_ms(1000)


    def run(self):#运行功能
        # 获取图像并进行二值化处理
        img = sensor.snapshot()
        base_speed = 0.1
        # 记录寻找到的黑线块
        blob_roi1 = 0  # 记录最大的块
        blob_roi2 = 0
        blob_roi3 = 0
        blob_roi4 = 0
        acute_crossing_flag = 0  # 路口判断
        weight_sum = 0  # 权值和初始化
        centroid_sum = 0

        if self.crossing_cnt >= 23:#终点
            self.robot_move_cmd.car_move(0, 0, 0, 0)
            time.sleep_ms(100)
            return

        # 利用颜色识别分别寻找三个矩形区域内的线段
        for r in self.ROIS:
            blobs = img.find_blobs(self.GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True)
            # 目标区域找到直线
            if blobs:
                # 查找像素最多的blob的索引。
                largest_blob = -1
                second_largest_blob = -1
                most_pixels = 0
                second_most_pixels = 0
                for i in range(len(blobs)):
                    # 目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                    if blobs[i].pixels() > most_pixels:
                        most_pixels = blobs[i].pixels()
                        largest_blob = i
                for i in range(len(blobs)):
                    # 目标区域找到的颜色块（线段块）可能不止一个，找到第二大的一个
                    if abs(blobs[i].x() - blobs[largest_blob].x()) > 20:  # 先排除一定范围的点
                        if blobs[i].pixels() > second_most_pixels and blobs[i].pixels() < most_pixels:
                            second_most_pixels = blobs[i].pixels()
                            second_largest_blob = i

                if blobs[largest_blob].w() > 10 and blobs[largest_blob].h() > 5:  # 太小说明检测到虚线之类的,略过
                    # 在色块周围画一个矩形。
                    img.draw_rectangle(blobs[largest_blob].rect())
                    img.draw_cross(blobs[largest_blob].cx(),
                                   blobs[largest_blob].cy())  # 将此区域的像素数最大的颜色块画矩形和十字形标记出来
                    img.draw_string(blobs[largest_blob].x(), blobs[largest_blob].y() - 10,
                                    "W:{} H:{}".format(blobs[largest_blob].w(), blobs[largest_blob].h()),
                                    color=(255, 255, 255))

                    if r[5] == 1:
                        blob_roi1 = blobs[largest_blob].rect()
                    elif r[5] == 2:
                        blob_roi2 = blobs[largest_blob].rect()
                    elif r[5] == 3:
                        blob_roi3 = blobs[largest_blob].rect()
                    elif r[5] == 4:
                        blob_roi4 = blobs[largest_blob].rect()

                if second_largest_blob != -1 and blobs[second_largest_blob].w() > 10 and blobs[
                    second_largest_blob].h() > 5:
                    # 在色块周围画一个矩形。
                    img.draw_rectangle(blobs[second_largest_blob].rect())
                    img.draw_cross(blobs[second_largest_blob].cx(),
                                   blobs[second_largest_blob].cy())  # 将此区域的像素数第二大的颜色块画矩形和十字形标记出来
                    img.draw_string(blobs[second_largest_blob].x(), blobs[second_largest_blob].y() - 10,
                                    "W:{} H:{}".format(blobs[second_largest_blob].w(), blobs[second_largest_blob].h()),
                                    color=(255, 0, 0))
                    acute_crossing_flag += 1
                # print("acute_crossing_flag:", acute_crossing_flag)

                ##print(blobs[largest_blob].rect())
                centroid_sum += blobs[largest_blob].cx() * r[4]  # r[4] is the roi weight.
                weight_sum += r[4]

        if acute_crossing_flag >= 2 and self.crossing_cnt >= 3 and self.crossing_cnt < 20:  # 发现锐角，右转
            self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)# 前进
            time.sleep_ms(2500)
            self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)# 右转
            time.sleep_ms(1500)
            self.crossing_flag = 0
            self.crossing_cnt = 21
            return
        # print((blob_roi1[0]+blob_roi1[2])/2,(blob_roi2[0]+blob_roi2[2])/2,(blob_roi3[0]+blob_roi3[2])/2,weight_sum)

        # **********************************判断路口情况，检测两个自定义的范围都检测到路口就说明经过一个路口***********************************************
        if self.crossing_flag == 0 and ((blob_roi3 != 0 and blob_roi3[2] > 80) or (blob_roi4 != 0 and blob_roi4[2] > 80)):
            self.crossing_flag = 1
        elif self.crossing_flag == 1 and blob_roi1 != 0 and blob_roi1[2] > 80:  # 1号ROIS检测到路口
            self.crossing_flag = 2

        if self.crossing_flag == 2:  # 找到路口
            self.crossing_flag = 0
            self.crossing_cnt += 1
            time.sleep_ms(20)
            if self.crossing_cnt == 1:  # 第1个路口小车需要右转
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)# 前进
                time.sleep_ms(2500)
                self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)# 右转
                time.sleep_ms(1500)
                return
            elif self.crossing_cnt == 2:  # 第2个路口小车需要左转
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1) # 前进
                time.sleep_ms(2500)
                self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)# 左转
                time.sleep_ms(1500)
                return
            elif self.crossing_cnt == 3:  # 第3个路口小车需要右转
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)
                time.sleep_ms(2500)
                self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)
                time.sleep_ms(1500)
                return
            elif self.crossing_cnt == 22:  # 第22个路口小车需要左转
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1) # 前进
                time.sleep_ms(2500)
                self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)# 左转
                time.sleep_ms(1500)
                return
            elif self.crossing_cnt == 23:  # 第23个路口小车需要左转
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1) # 前进
                time.sleep_ms(2500)
                self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)# 左转
                time.sleep_ms(1500)
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1) # 前进
                time.sleep_ms(3500)
                return

        if weight_sum > 0:
            center_pos = (centroid_sum / weight_sum)  # Determine center of line.

            # 将center_pos转换为一个偏角。我们用的是非线性运算，所以越偏离直线，响应越强。
            # 非线性操作很适合用于这样的算法的输出，以引起响应“触发器”。
            deflection_angle = 0
            # 机器人应该转的角度

            # 80是X的一半，60是Y的一半。
            # 下面的等式只是计算三角形的角度，其中三角形的另一边是中心位置与中心的偏差，相邻边是Y的一半。
            # 这样会将角度输出限制在-45至45度左右。（不完全是-45至45度）。

            deflection_angle = -math.atan((center_pos - 80) / 60)
            # 角度计算.80 60 分别为图像宽和高的一半，图像大小为QQVGA 160x120.
            # 注意计算得到的是弧度值

            deflection_angle = math.degrees(deflection_angle)
            # 将计算结果的弧度值转化为角度值

            # 现在你有一个角度来告诉你该如何转动机器人。
            # 通过该角度可以合并最靠近机器人的部分直线和远离机器人的部分直线，以实现更好的预测。

            #print("Turn Angle:", deflection_angle)
            if deflection_angle < 0:  # 右转
                # 右侧电机速度增加，左侧电机速度减少
                self.speed_motor1 = base_speed * (1 + abs(deflection_angle) / 45)
                self.speed_motor2 = base_speed * (1 - abs(deflection_angle) / 20)
                self.speed_motor3 = base_speed * (1 + abs(deflection_angle) / 45)
                self.speed_motor4 = base_speed * (1 - abs(deflection_angle) / 20)
            else:  # 左转
                # 左侧电机速度增加，右侧电机速度减少
                self.speed_motor1 = base_speed * (1 - abs(deflection_angle) / 20)
                self.speed_motor2 = base_speed * (1 + abs(deflection_angle) / 45)
                self.speed_motor3 = base_speed * (1 - abs(deflection_angle) / 20)
                self.speed_motor4 = base_speed * (1 + abs(deflection_angle) / 45)

            self.robot_move_cmd.car_move(self.speed_motor1, self.speed_motor2, self.speed_motor3, self.speed_motor4)
        else:
            self.robot_move_cmd.car_move(0, 0, 0, 0)
            time.sleep_ms(100)


if __name__ == "__main__":
    app=LineMap3()
    app.init()#初始化

    while(1):
        app.run()#运行功能








