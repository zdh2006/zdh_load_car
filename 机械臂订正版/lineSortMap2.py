import sensor, time, math
from pyb import Pin,Timer,UART
import kinematics, robotMoveCmd

class LineSortMap2():
    # 设置阈值，如果是黑线，GRAYSCALE_THRESHOLD = [(0, 64)]；
    # 如果是白线，GRAYSCALE_THRESHOLD = [(128，255)]
    GRAYSCALE_THRESHOLD = [(0, 64)]

    # 每个roi为(x, y, w, h)，线检测算法将尝试找到每个roi中最大的blob的质心。
    # 然后用不同的权重对质心的x位置求平均值，其中最大的权重分配给靠近图像底部的roi，
    # 较小的权重分配给下一个roi，以此类推。
    ROIS = [  # [ROI, weight]
        (0, 100, 200, 20, 0.5, 1),  # 你需要为你的应用程序调整权重
        (0, 75, 200, 20, 0.2, 2),
        (0, 50, 200, 20, 0.2, 3),  # 取决于你的机器人是如何设置的。
        (0, 25, 200, 20, 0.05, 4),
        (0, 000, 200, 20, 0.05, 5)
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

    #物块中心点
    mid_block_cx=80
    mid_block_cy=60

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

        self.adjust_position = 0  # 用来判断小车状态
        self.cap_block_cnt = 0  # 用来计数抓取物块数量
        self.crossing_flag = 0  # 标记路口情况计数，判断是否经过一个路口
        self.is_line_flag = 1  # 是否可以巡线标志
        self.cap_color_status = 0  # 抓取物块颜色标志，用来判断物块抓取
        self.move_status = 0  # 机械臂移动的方式
        self.crossing_record_cnt = 0  # 用来记录经过的路口数量
        self.mid_adjust_position = 0  # 小车到中间横线时需要调整身位后在寻找分拣区，变量为标志位
        self.mid_block_cnt = 0  # 用来记录机械臂已对准物块计数，防止误差
        self.over_flag = 0  # 用来标记小车180度翻转
        self.mid_over_flag = 0  # 记录小车翻转到一半
        self.mid_over_cnt = 0  # 记录小车翻转到一半计数
        self.adjust_position_cnt = 0  # 调整身位计数
        self.car_back_flag = 0  # 小车后退还是前进标志

        self.crossing_cnt = 0  # MAP3路口计数
        self.speed_motor1 = 0
        self.speed_motor2 = 0
        self.speed_motor3 = 0
        self.speed_motor4 = 0

        self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
        time.sleep_ms(1000)

    def line_walk(self, img):  # 巡线功能,传入img是二值化的图像
        weight_sum = 0  # 权值和初始化
        centroid_sum = 0

        # 计算基础速度
        base_speed = 0.1  # 基础速度，可以根据需要调整

        # 记录寻找到的黑线块
        blob_roi1 = 0  # 记录最大的块
        blob_roi2 = 0
        blob_roi3 = 0
        blob_roi4 = 0
        blob_roi5 = 0

        # 利用颜色识别分别寻找三个矩形区域内的线段
        for r in self.ROIS:
            blobs = img.find_blobs(self.GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True)
            # 目标区域找到直线
            if blobs:
                # 查找像素最多的blob的索引。
                largest_blob = 0
                most_pixels = 0
                for i in range(len(blobs)):
                    # 目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                    if blobs[i].pixels() > most_pixels:
                        most_pixels = blobs[i].pixels()
                        largest_blob = i
                if blobs[largest_blob].w() > 10 and blobs[largest_blob].h() > 5:  # 太小说明检测到虚线之类的,略过
                    # 在色块周围画一个矩形。
                    img.draw_rectangle(blobs[largest_blob].rect())
                    img.draw_cross(blobs[largest_blob].cx(),
                                   blobs[largest_blob].cy())  # 将此区域的像素数最大的颜色块画矩形和十字形标记出来
                    img.draw_string(blobs[largest_blob].x(), (blobs[largest_blob].y()), "{}".format(r[5]),
                                    color=(255, 255, 255))

                    if r[5] == 1:
                        blob_roi1 = blobs[largest_blob].rect()
                    elif r[5] == 2:
                        blob_roi2 = blobs[largest_blob].rect()
                    elif r[5] == 3:
                        blob_roi3 = blobs[largest_blob].rect()
                    elif r[5] == 4:
                        blob_roi4 = blobs[largest_blob].rect()
                    elif r[5] == 5:
                        blob_roi5 = blobs[largest_blob].rect()

                    centroid_sum += blobs[largest_blob].cx() * r[4]  # r[4] is the roi weight.
                    weight_sum += r[4]

        if self.over_flag == 1:  # 原地翻转
            if blob_roi3 == 0 and blob_roi5 == 0 and self.mid_over_flag == 0:  # 摄像头已经识别不到线，说明翻转到一半
                self.mid_over_cnt += 1
                if self.mid_over_cnt > 5:
                    self.mid_over_flag = 1
                    self.mid_over_cnt = 0
            elif blob_roi1 != 0 and blob_roi3 != 0 and blob_roi5 != 0 and self.mid_over_flag == 1:  # 重新识别到三条范围内的线，取消翻转，重新开始巡线
                self.over_flag = 0
                self.mid_over_flag = 0
                time.sleep_ms(200)
            self.robot_move_cmd.car_move(-0.1, 0.1, -0.1, 0.1)
            return

        # **********************************判断路口情况，检测两个自定义的范围都检测到路口就说明经过一个路口***********************************************
        if self.crossing_flag == 0 and (
                (blob_roi3 != 0 and blob_roi3[2] > 90) or (blob_roi4 != 0 and blob_roi4[2] > 90)):
            self.crossing_flag = 1
        elif self.crossing_flag == 1 and blob_roi1 != 0 and blob_roi1[2] > 90:  # 1号ROIS检测到路口
            self.crossing_flag = 2

        if weight_sum > 0 and self.is_line_flag == 1:  # 如果识别到线条或者没有特殊情况，开始巡线

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

            # print("Turn Angle: %f" % deflection_angle)\

            if self.mid_adjust_position == 1:  # 调整身位
                if blob_roi5 != 0 and blob_roi5[2] > 100 and abs(deflection_angle) < 5:  # 身位调整完毕
                    self.adjust_position_cnt += 1
                    self.robot_move_cmd.car_move(0, 0, 0, 0)
                    if self.adjust_position_cnt > 10:  # 调整身位成功
                        self.adjust_position_cnt = 0
                        self.is_line_flag = 0
                        self.car_back_flag = 0
                        self.mid_adjust_position = 0
                        self.move_x = 120  # 旋转机械臂寻找颜色框
                        if self.crossing_record_cnt == 3:  # 第三个路口的机械臂旋转的方向不同
                            self.move_x = -120
                        self.move_y = 50
                        self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)
                        time.sleep_ms(1200)
                    return
                # 小车身位太靠前，需后退
                elif blob_roi5 == 0 or (blob_roi4 != 0 and blob_roi4[2] > 100) or (
                        blob_roi3 != 0 and blob_roi3[2] > 100) or (blob_roi2 != 0 and blob_roi2[2] > 100) or (
                        blob_roi1 != 0 and blob_roi1[2] > 100):
                    self.car_back_flag = 1
                elif blob_roi5 != 0 and blob_roi5[2] < 50:  # 小车倒退出线
                    self.car_back_flag = 0
                self.adjust_position_cnt = 0

            if self.mid_adjust_position == 1 or self.car_back_flag == 1:  # map2的地图调整
                if self.car_back_flag == 1 and self.crossing_record_cnt != 12:  # 小车需倒退
                    if deflection_angle < -6:  # 右转
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 2
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 2
                    elif deflection_angle > 6:  # 左
                        self.speed_motor1 = -base_speed / 2
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 2
                        self.speed_motor4 = -base_speed / 3
                    else:
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 3
                elif self.car_back_flag == 0 and self.crossing_record_cnt != 12:
                    # 计算每个电机的速度，根据偏转角度进行调整
                    if deflection_angle < -10:  # 右转
                        self.speed_motor1 = base_speed / 3
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = base_speed / 3
                        self.speed_motor4 = -base_speed / 3
                    elif deflection_angle > 10:  # 左转
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = base_speed / 3
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = base_speed / 3
                    else:
                        self.speed_motor1 = base_speed / 3
                        self.speed_motor2 = base_speed / 3
                        self.speed_motor3 = base_speed / 3
                        self.speed_motor4 = base_speed / 3
                elif self.car_back_flag == 1 and self.crossing_record_cnt == 12:
                    if deflection_angle < -20:  # 右转
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 2
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 2
                    elif deflection_angle > 20:  # 左
                        self.speed_motor1 = -base_speed / 2
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 2
                        self.speed_motor4 = -base_speed / 3
                    else:
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 3

            else:
                if deflection_angle < 0:  # 右转
                    # 右侧电机速度增加，左侧电机速度减少
                    self.speed_motor1 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor2 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor3 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor4 = base_speed * (1 - abs(deflection_angle) / 45)
                else:  # 左转
                    # 左侧电机速度增加，右侧电机速度减少
                    self.speed_motor1 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor2 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor3 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor4 = base_speed * (1 + abs(deflection_angle) / 45)

            self.robot_move_cmd.car_move(self.speed_motor1, self.speed_motor2, self.speed_motor3, self.speed_motor4)

        # ************************************************ 出线暂停*************************************************************************************
        else:
            self.robot_move_cmd.car_move(0, 0, 0, 0)
            time.sleep_ms(500)


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
        color_read_succeed = 0  # 是否识别到颜色
        color_status = 0

        # 获取图像
        img = sensor.snapshot()

        if self.is_line_flag == 1:
            img = img.to_grayscale()
            self.line_walk(img)
        else:
            red_blobs = img.find_blobs([self.red_threshold], x_stride=15, y_stride=15, pixels_threshold=25)
            blue_blobs = img.find_blobs([self.blue_threshold], x_stride=15, y_stride=15, pixels_threshold=25)
            green_blobs = img.find_blobs([self.green_threshold], x_stride=15, y_stride=15, pixels_threshold=25)

            # ***************首先进行色块检测，如果没有检测到色块，那就寻线********************
            if red_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'R'):  # 红色
                color_status = 'R'
                color_read_succeed = 1
                max_size = 0
                max_blob = 0
                for blob in red_blobs:#寻找最大
                    if blob[2] * blob[3] > max_size:
                        max_blob = blob
                        max_size = blob[2] * blob[3]
                img.draw_rectangle((max_blob[0],max_blob[1],max_blob[2],max_blob[3]),color=(255,255,255))
                img.draw_cross(max_blob[5], max_blob[6],size=2,color=(255,0,0))
                img.draw_string(max_blob[0], (max_blob[1]-10), "red", color=(255,0,0))
                block_cx=max_blob[5]
                block_cy=max_blob[6]
            elif blue_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'B'):  # 蓝色
                color_status = 'B'
                color_read_succeed = 1
                max_size = 0
                max_blob = 0
                for blob in blue_blobs:#寻找最大
                    if blob[2] * blob[3] > max_size:
                        max_blob = blob
                        max_size = blob[2] * blob[3]
                img.draw_rectangle((max_blob[0],max_blob[1],max_blob[2],max_blob[3]),color=(255,255,255))
                img.draw_cross(max_blob[5], max_blob[6],size=2,color=(0,0,255))
                img.draw_string(max_blob[0], (max_blob[1]-10), "blue", color=(0,0,255))
                block_cx=max_blob[5]
                block_cy=max_blob[6]
            elif green_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'G'):  # 绿色
                color_status = 'G'
                color_read_succeed = 1
                max_size = 0
                max_blob = 0
                for blob in green_blobs:#寻找最大
                    if blob[2] * blob[3] > max_size:
                        max_blob = blob
                        max_size = blob[2] * blob[3]
                img.draw_rectangle((max_blob[0],max_blob[1],max_blob[2],max_blob[3]),color=(255,255,255))
                img.draw_cross(max_blob[5], max_blob[6],size=2,color=(255,0,0))
                img.draw_string(max_blob[0], (max_blob[1]-10), "green", color=(255,0,0))
                block_cx=max_blob[5]
                block_cy=max_blob[6]

        # **********************************判断路口情况***********************************************
        if self.crossing_flag == 2:  # 找到路口
            self.crossing_flag = 0
            self.crossing_record_cnt += 1
            if self.crossing_record_cnt == 2 or self.crossing_record_cnt == 5 or self.crossing_record_cnt == 9:  # 经过的路口数量在物品区，小车停止，开始颜色识别
                self.is_line_flag = 0
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)# 前进
                time.sleep_ms(500)
                self.robot_move_cmd.car_move(0, 0, 0, 0)
            elif self.crossing_record_cnt == 3:  # 第3个路口小车需要右转,改为颜色识别，旋转机械臂到左边
                self.mid_adjust_position = 1
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)# 前进
                time.sleep_ms(2500)
                self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)# 右转
                time.sleep_ms(1500)
            elif self.crossing_record_cnt == 4 or self.crossing_record_cnt == 11:  # 第4,11个路口小车需要右转
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)# 前进
                time.sleep_ms(2500)
                self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)# 右转
                time.sleep_ms(1500)
            elif self.crossing_record_cnt == 6:  # 第6个路口小车需要左转,改为颜色识别，旋转机械臂到右边
                self.mid_adjust_position = 1
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1) # 前进
                time.sleep_ms(2500)
                self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)# 左转
                time.sleep_ms(1500)
            elif self.crossing_record_cnt == 7:  # 第7个路口小车需要旋转180度
                self.robot_move_cmd.car_move(-0.1, 0.1, -0.1, 0.1)
                time.sleep_ms(1000)
                self.over_flag = 1
                return
            elif self.crossing_record_cnt == 8:  # 第8个路口小车需要左转
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1) # 前进
                time.sleep_ms(2500)
                self.robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)# 左转
                time.sleep_ms(1500)
            elif self.crossing_record_cnt == 10:  # 第10个路口小车需要右转,改为颜色识别，旋转机械臂到右边
                self.mid_adjust_position = 1
                self.robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)# 前进
                time.sleep_ms(2500)
                self.robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)# 右转
                time.sleep_ms(1500)
            elif self.crossing_record_cnt == 12:  # 第12个路口小车回到原点，需要原地旋转180度后倒车
                self.robot_move_cmd.car_move(0, 0, 0, 0)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 500)
                time.sleep_ms(600)
                self.crossing_flag = 1
                self.car_back_flag = 1
                self.over_flag = 1
                return
            elif self.crossing_record_cnt == 13:  # 第13个路口小车回到原点:
                self.robot_move_cmd.car_move(0, 0, 0, 0)
                self.is_line_flag = 0
                return

        # ************************************************ 运动机械臂*************************************************************************************
        if color_read_succeed == 1:  # 识别到颜色或者到路口
            if self.move_status == 0:  # 第0阶段：机械臂寻找物块位置
                if (abs(block_cx - self.mid_block_cx) > 3):
                    if block_cx > self.mid_block_cx:
                        self.move_x += 0.5
                    else:
                        self.move_x -= 0.5
                if (abs(block_cy - self.mid_block_cy) > 3):
                    if block_cy > self.mid_block_cy and self.move_y > 1:
                        self.move_y -= 0.3
                    else:
                        self.move_y += 0.3
                if abs(block_cy - self.mid_block_cy) <= 3 and abs(block_cx - self.mid_block_cx) <= 3:  # 寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 10:  # 计数10次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 1
                        self.cap_color_status = color_status
                else:
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 0)
                time.sleep_ms(10)

            elif self.move_status == 1:  # 第1阶段：机械臂抓取物块
                self.move_status = 2
                l = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = (l + 79+cy) * cos+cx
                self.move_y = (l + 79+cy) * sin
                time.sleep_ms(100)
                self.kinematic.send_str("{#005P1000T1000!}")
                time.sleep_ms(100)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  # 移动机械臂到物块上方
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, -75+cz, 1000)  # 移动机械臂下移到物块
                time.sleep_ms(1200)
                self.kinematic.send_str("{#005P1700T1000!}")  # 机械爪抓取物块
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  # 移动机械臂抬起
                time.sleep_ms(1200)
                self.move_x = 0  # 机械臂归位
                self.move_y = 160
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep_ms(1200)
                self.crossing_flag = 0
                self.mid_block_cnt = 0
                self.is_line_flag = 1  # 机器人开始巡线
                self.over_flag = 1  # 原地翻转标志

            elif self.move_status == 2:  # 第2阶段：调整身位
                if block_cx - self.mid_block_cx > 20:
                    if self.crossing_record_cnt == 3:  # 路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                        if self.adjust_position != 1:
                            self.adjust_position = 1
                            self.robot_move_cmd.car_move(0.05, 0.05, 0.05, 0.05)  # 前进
                    else:
                        if self.adjust_position != 4:
                            self.adjust_position = 4
                            self.robot_move_cmd.car_move(-0.05, -0.05, -0.05, -0.05)  # 后退
                    return

                elif block_cx - self.mid_block_cx < -20:
                    if self.crossing_record_cnt == 3:  # 路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                        if self.adjust_position != 4:
                            self.adjust_position = 4
                            self.robot_move_cmd.car_move(-0.05, -0.05, -0.05, -0.05)  # 后退
                    else:
                        if self.adjust_position != 1:
                            self.adjust_position = 1
                            self.robot_move_cmd.car_move(0.05, 0.05, 0.05, 0.05)  # 前进
                    return
                else:  # 调整完毕，停止
                    if self.adjust_position != 5:
                        self.adjust_position = 5
                        self.robot_move_cmd.car_move(0, 0, 0, 0)
                        self.move_status = 3

            elif self.move_status == 3:  # 第3阶段：机械臂寻找放下物块的框框
                if abs(block_cx - self.mid_block_cx) > 5:
                    if block_cx > self.mid_block_cx and self.move_y > 1:
                        if self.crossing_record_cnt == 3:  # 路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                            self.move_y += 0.5
                        else:
                            self.move_y -= 0.5
                    else:
                        if self.crossing_record_cnt == 3:
                            self.move_y -= 0.5
                        else:
                            self.move_y += 0.5
                if abs(block_cy - self.mid_block_cy) > 5:
                    if block_cy > self.mid_block_cy:
                        if self.crossing_record_cnt == 3:
                            self.move_x += 0.3
                        else:
                            self.move_x -= 0.3
                    else:
                        if self.crossing_record_cnt == 3:
                            self.move_x -= 0.3
                        else:
                            self.move_x += 0.3
                if abs(block_cy - self.mid_block_cy) <= 5 and abs(block_cx - self.mid_block_cx) <= 5:  # 寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 10:  # 计数10次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 4
                else:
                    self.mid_block_cnt = 0
                    self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 0)
                time.sleep_ms(10)

            elif self.move_status == 4:  # 第4阶段：机械臂放下物块并归位
                self.move_status = 0
                l = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                sin = self.move_y / l
                cos = self.move_x / l
                self.move_x = (l + 86 + cy) * cos + cx
                self.move_y = (l + 86 + cy) * sin
                self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)  # 移动机械臂到物块上方
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, -50+cz, 1000)  # 移动机械臂下移到物块
                time.sleep_ms(1200)
                self.kinematic.send_str("{#005P1000T1000!}")  # 机械爪放下物块
                time.sleep_ms(1200)
                self.kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)  # 移动机械臂抬起
                time.sleep_ms(1200)
                self.move_x = 0  # 机械臂归位
                self.move_y = 160
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep_ms(1200)
                self.mid_block_cnt = 0
                self.is_line_flag = 1  # 机器人开始巡线
                self.cap_color_status = 0
                self.adjust_position = 0

if __name__ == "__main__":
    app=LineSortMap2()
    app.init()#初始化

    while(1):
        app.run(0,0,0)#运行功能








