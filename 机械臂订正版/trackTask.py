"""
==========================================================================
  文件名: trackTask.py
  功能:   C题"寻迹小车自动装卸系统"的核心任务文件
         整合巡线(OpenMV识别黑线) + 识别红色边界框(仓库区/卸载区)
         + 识别物块颜色(A/B/C) + 机械臂抓取/放置
  说明:
    - 巡线时, 机械臂保持固定低头姿态, 让摄像头以固定角度朝下看地面黑线
    - 识别到红色边界框(卸载区/仓库区)时, 小车停下
    - 到仓库区时: 机械臂逐个识别并抓取A/B/C三个物块, 放到车身上(码垛)
    - 到卸载区时: 根据STM32发来的卸载顺序, 机械臂转向侧面放下对应物块
    - 全程通过UART3与STM32通信:
        H7→STM32: "$Car:v1,v2,v3,v4!" 控制电机
        STM32→H7: "#Order:ABC!" 传递卸载顺序
                   "#Go!"       启动任务
==========================================================================
"""


import sensor, time, math
from pyb import Pin, Timer, UART
import kinematics, robotMoveCmd

class TrackTask():
    # ========================== 颜色阈值(需根据实际光照调整) ==========================
    # 黑线阈值(灰度图下使用)
    GRAYSCALE_THRESHOLD = [(0, 64)]

    # 红色阈值(LAB空间) — 用于识别红色边界框(启停区/仓库区/卸载区边框)
    red_threshold   = (0, 100, 20, 127, 0, 127)

    # A/B/C三种物块的颜色阈值 — 你需要根据实际物块颜色修改
    # 这里假设: A=红色物块, B=绿色物块, C=蓝色物块
    # 如果你的A/B/C用其他颜色区分, 改这里就行
    color_A_threshold = (0, 100, 20, 127, 0, 127)     # 物块A的颜色(红色)
    color_B_threshold = (0, 100, -128, -28, 0, 70)     # 物块B的颜色(绿色)
    color_C_threshold = (0, 60, -128, 127, -128, -28)   # 物块C的颜色(蓝色)

    # ========================== 巡线ROI区域 ==========================
    # 每个ROI: (x, y, w, h, 权重, 编号)
    # QQVGA = 160x120, ROI把图像分成5个横条
    # 靠近底部(靠近小车)的权重大, 远处的权重小
    ROIS = [
        (0, 100, 160, 20, 0.7, 1),   # 最近(底部) — 权重最大
        (0,  75, 160, 20, 0.3, 2),
        (0,  50, 160, 20, 0.2, 3),
        (0,  25, 160, 20, 0.1, 4),   # 最远(顶部) — 权重最小
    ]

    # ========================== 串口 ==========================
    uart = UART(3, 115200)
    uart.init(115200, bits=8, parity=None, stop=1)

    # ========================== 补光灯 ==========================
    tim = Timer(4, freq=1000)
    led_dac = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=0)

    # ========================== 运动控制对象 ==========================
    robot_move_cmd = robotMoveCmd.RobotMoveCmd()
    kinematic = kinematics.Kinematics()

    # ========================== 机械臂巡线姿态参数 ==========================
    # 巡线时机械臂的固定位置, 让摄像头低头看地面
    # 这两个值决定了摄像头的朝向角度, 需要根据你的机械臂实际安装位置调整
    LINE_FOLLOW_X = 0       # 巡线时机械臂X坐标(正前方=0)
    LINE_FOLLOW_Y = 100     # 巡线时机械臂Y坐标(越小=越低头)
    LINE_FOLLOW_Z = 80      # 巡线时机械臂Z坐标(高度)

    # ========================== 机械臂抓取/放置姿态参数 ==========================
    # 仓库区抓取物块时, 机械臂伸到正前方
    GRAB_X = 0
    GRAB_Y = 160
    GRAB_Z = 50

    # 物块放到车身上(码垛位置) — 机械臂收回到车身上方
    STACK_X = 0
    STACK_Y = 80
    STACK_Z = 50

    # 卸载时机械臂伸到右侧(卸载区在行进方向右边)
    UNLOAD_X = 140
    UNLOAD_Y = 60
    UNLOAD_Z = 50

    # ========================== 巡线基础速度 ==========================
    BASE_SPEED = 0.12   # 巡线基础速度(m/s), 根据实际调快调慢

    def __init__(self):
        """构造函数: 初始化所有状态变量"""
        # ---------- 任务总状态 ----------
        # 0=等待启动, 1=巡线去仓库, 2=仓库装货,
        # 3=巡线去卸载区, 4=卸货, 5=巡线回启停区, 6=任务完成
        self.task_state = 0

        # ---------- 卸载顺序 ----------
        # 默认A→B→C, STM32可通过串口修改
        # 列表中存的是字符 'A','B','C'
        self.unload_order = ['A', 'B', 'C']

        # ---------- 物块相关 ----------
        self.grab_count = 0          # 已抓取物块数量(0~3)
        self.unload_count = 0        # 已卸载物块数量(0~3)
        self.grabbed_blocks = []     # 已抓到的物块列表, 如['A','B','C']

        # ---------- 巡线相关 ----------
        self.crossing_flag = 0       # 路口检测状态(0=未检测, 1=远处检测到宽线, 2=近处也检测到=确认路口)
        self.crossing_cnt = 0        # 经过的路口计数
        self.red_frame_flag = 0      # 红色边界框检测标志

        # ---------- 机械臂相关 ----------
        self.move_x = 0
        self.move_y = 160
        self.move_z = 50
        self.arm_state = 0           # 机械臂子状态机
        self.mid_block_cnt = 0       # 对准物块计数(防抖)
        self.mid_block_cx = 80       # 画面中心X (QQVGA宽160/2=80)
        self.mid_block_cy = 60       # 画面中心Y (QQVGA高120/2=60)

        # ---------- 小车调整 ----------
        self.adjust_position = 0

    def init(self):
        """初始化摄像头、补光灯、机械臂到巡线姿态"""
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QQVGA)       # 160x120
        sensor.skip_frames(n=2000)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)

        # 打开补光灯(0~100, 根据需要调)
        self.led_dac.pulse_width_percent(50)

        # 重置所有状态
        self.task_state = 0
        self.grab_count = 0
        self.unload_count = 0
        self.grabbed_blocks = []
        self.crossing_flag = 0
        self.crossing_cnt = 0
        self.red_frame_flag = 0
        self.arm_state = 0
        self.mid_block_cnt = 0
        self.adjust_position = 0
        self.unload_order = ['A', 'B', 'C']

        # 机械臂移动到巡线姿态(低头看地面)
        self.move_x = self.LINE_FOLLOW_X
        self.move_y = self.LINE_FOLLOW_Y
        self.move_z = self.LINE_FOLLOW_Z
        self.kinematic.kinematics_move(self.move_x, self.move_y, self.move_z, 1000)
        time.sleep_ms(1500)

        print("[INIT] TrackTask initialized, waiting for #Go! command...")

    # ==========================================================================
    #                           巡线功能
    # ==========================================================================
    def line_follow(self, img_gray):
        """
        巡线函数: 输入灰度图, 通过加权质心法计算偏转角, 控制小车左右转向
        同时检测路口(横向宽黑线)
        返回值: deflection_angle(偏转角), 供调用者判断
        """
        weight_sum = 0.0
        centroid_sum = 0.0
        blob_roi1 = None  # 最近的ROI检测结果
        blob_roi3 = None  # 中间ROI
        blob_roi4 = None  # 最远ROI

        for r in self.ROIS:
            blobs = img_gray.find_blobs(self.GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True)
            if blobs:
                # 找面积最大的blob
                largest_blob = max(blobs, key=lambda b: b.pixels())

                if largest_blob.w() > 8 and largest_blob.h() > 4:
                    img_gray.draw_rectangle(largest_blob.rect())
                    img_gray.draw_cross(largest_blob.cx(), largest_blob.cy())

                    # 记录各ROI的检测结果
                    if r[5] == 1:
                        blob_roi1 = largest_blob.rect()
                    elif r[5] == 3:
                        blob_roi3 = largest_blob.rect()
                    elif r[5] == 4:
                        blob_roi4 = largest_blob.rect()

                    centroid_sum += largest_blob.cx() * r[4]
                    weight_sum += r[4]

        # -------- 路口检测(横向宽黑线 = 路口/边界框位置) --------
        if self.crossing_flag == 0:
            # 远处ROI先检测到宽线(宽度>90像素)
            if (blob_roi3 and blob_roi3[2] > 90) or (blob_roi4 and blob_roi4[2] > 90):
                self.crossing_flag = 1
        elif self.crossing_flag == 1:
            # 近处ROI也检测到宽线, 确认经过路口
            if blob_roi1 and blob_roi1[2] > 90:
                self.crossing_flag = 2

        # -------- 计算偏转角并控制电机 --------
        if weight_sum > 0:
            center_pos = centroid_sum / weight_sum
            # 计算偏转角(度), 80=图像宽度一半, 60=高度一半
            deflection_angle = -math.atan((center_pos - 80) / 60)
            deflection_angle = math.degrees(deflection_angle)

            # 差速转向
            if deflection_angle < 0:  # 需要右转
                speed_l = self.BASE_SPEED * (1 + abs(deflection_angle) / 45)
                speed_r = self.BASE_SPEED * (1 - abs(deflection_angle) / 30)
            else:  # 需要左转
                speed_l = self.BASE_SPEED * (1 - abs(deflection_angle) / 30)
                speed_r = self.BASE_SPEED * (1 + abs(deflection_angle) / 45)

            self.robot_move_cmd.car_move(speed_l, speed_r, speed_l, speed_r)
            return deflection_angle
        else:
            # 完全丢线, 停车
            self.robot_move_cmd.car_move(0, 0, 0, 0)
            return 0

    # ==========================================================================
    #                           红框检测
    # ==========================================================================
    def detect_red_frame(self, img_rgb):
        """
        在RGB图像中检测红色边界框
        返回: True=检测到红色区域(面积足够大), False=未检测到
        """
        red_blobs = img_rgb.find_blobs([self.red_threshold],
                                        x_stride=10, y_stride=10,
                                        pixels_threshold=80)
        if red_blobs:
            largest = max(red_blobs, key=lambda b: b.pixels())
            if largest.pixels() > 150:  # 面积阈值, 根据实际调整
                img_rgb.draw_rectangle(largest.rect(), color=(255, 0, 0))
                img_rgb.draw_string(largest.x(), largest.y() - 10, "RED", color=(255, 0, 0))
                return True
        return False

    # ==========================================================================
    #                       物块颜色识别
    # ==========================================================================
    def detect_block_color(self, img_rgb):
        """
        在RGB图像中识别物块颜色
        返回: ('A', cx, cy) 或 ('B', cx, cy) 或 ('C', cx, cy) 或 None
        优先级: 按照还没抓到的物块来识别
        """
        results = []

        # 检测A颜色
        blobs_a = img_rgb.find_blobs([self.color_A_threshold],
                                      x_stride=15, y_stride=15, pixels_threshold=30)
        if blobs_a:
            largest = max(blobs_a, key=lambda b: b.pixels())
            if largest.w() > 10 and largest.h() > 10:
                results.append(('A', largest.cx(), largest.cy(), largest.pixels()))

        # 检测B颜色
        blobs_b = img_rgb.find_blobs([self.color_B_threshold],
                                      x_stride=15, y_stride=15, pixels_threshold=30)
        if blobs_b:
            largest = max(blobs_b, key=lambda b: b.pixels())
            if largest.w() > 10 and largest.h() > 10:
                results.append(('B', largest.cx(), largest.cy(), largest.pixels()))

        # 检测C颜色
        blobs_c = img_rgb.find_blobs([self.color_C_threshold],
                                      x_stride=15, y_stride=15, pixels_threshold=30)
        if blobs_c:
            largest = max(blobs_c, key=lambda b: b.pixels())
            if largest.w() > 10 and largest.h() > 10:
                results.append(('C', largest.cx(), largest.cy(), largest.pixels()))

        if not results:
            return None

        # 过滤掉已经抓过的物块
        for r in results:
            if r[0] not in self.grabbed_blocks:
                return (r[0], r[1], r[2])

        return None

    # ==========================================================================
    #                     机械臂抓取物块(仓库区)
    # ==========================================================================
    def grab_block(self, img, cx=0, cy=0, cz=0):
        """
        在仓库区抓取物块的子状态机
        cx/cy/cz: 抓取偏移量微调参数
        返回: True=本次抓取完成, False=还在进行中
        """
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy

        # 识别物块
        detection = self.detect_block_color(img)
        if detection is None and self.arm_state < 2:
            # 没识别到物块, 微调小车位置(原地小幅旋转寻找)
            self.robot_move_cmd.car_move(-0.05, 0.05, -0.05, 0.05)
            time.sleep_ms(50)
            return False

        if detection:
            block_name, block_cx, block_cy = detection
            img.draw_cross(block_cx, block_cy, size=5, color=(255, 255, 0))
            img.draw_string(block_cx, block_cy - 15, block_name, color=(255, 255, 0))

        # ---- 子状态机 ----
        if self.arm_state == 0:
            # 阶段0: 机械臂对准物块(微调move_x, move_y使物块在画面中心)
            if detection is None:
                return False
            if abs(block_cx - self.mid_block_cx) > 3:
                if block_cx > self.mid_block_cx:
                    self.move_x += 0.3
                else:
                    self.move_x -= 0.3
            if abs(block_cy - self.mid_block_cy) > 3:
                if block_cy > self.mid_block_cy and self.move_y > 80:
                    self.move_y -= 0.3
                else:
                    self.move_y += 0.3

            if abs(block_cx - self.mid_block_cx) <= 3 and abs(block_cy - self.mid_block_cy) <= 3:
                self.mid_block_cnt += 1
                if self.mid_block_cnt > 10:
                    self.mid_block_cnt = 0
                    self.arm_state = 1  # 对准完成, 进入抓取
                    self.current_grab_name = block_name  # 记录当前要抓的物块名
            else:
                self.mid_block_cnt = 0
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 10)
            time.sleep_ms(10)

        elif self.arm_state == 1:
            # 阶段1: 执行抓取动作序列
            self.robot_move_cmd.car_move(0, 0, 0, 0)  # 确保小车停稳

            # 计算补偿后的目标位置
            l = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
            if l < 1:
                l = 1
            sin_val = self.move_y / l
            cos_val = self.move_x / l
            target_x = (l + 79 + cy) * cos_val + cx
            target_y = (l + 79 + cy) * sin_val

            # 张开爪子
            self.kinematic.send_str("{#005P1000T1000!}")
            time.sleep_ms(200)

            # 移动到物块上方
            self.kinematic.kinematics_move(target_x, target_y, 50, 1000)
            time.sleep_ms(1200)

            # 下降到物块
            self.kinematic.kinematics_move(target_x, target_y, -75 + cz, 1000)
            time.sleep_ms(1200)

            # 闭合爪子抓取
            self.kinematic.send_str("{#005P1700T1000!}")
            time.sleep_ms(1200)

            # 抬起
            self.kinematic.kinematics_move(target_x, target_y, 50, 1000)
            time.sleep_ms(1200)

            # 移到车身码垛位置
            stack_z = self.STACK_Z + self.grab_count * 30  # 每个物块叠高30mm
            self.kinematic.kinematics_move(self.STACK_X, self.STACK_Y, 70, 1000)
            time.sleep_ms(1200)
            self.kinematic.kinematics_move(self.STACK_X, self.STACK_Y, stack_z, 1000)
            time.sleep_ms(1200)

            # 松开爪子放下
            self.kinematic.send_str("{#005P1000T1000!}")
            time.sleep_ms(1000)

            # 抬起避让
            self.kinematic.kinematics_move(self.STACK_X, self.STACK_Y, 70, 1000)
            time.sleep_ms(1000)

            # 记录
            self.grabbed_blocks.append(self.current_grab_name)
            self.grab_count += 1

            # 发送当前识别信息给STM32显示
            self.uart.write("$Grab:{}!\n".format(self.current_grab_name))

            print("[GRAB] Grabbed block:", self.current_grab_name,
                  "Total:", self.grab_count)

            # 回到观察位置继续找下一个
            self.move_x = self.GRAB_X
            self.move_y = self.GRAB_Y
            self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
            time.sleep_ms(1200)

            self.arm_state = 0
            self.mid_block_cnt = 0

            if self.grab_count >= 3:
                return True   # 三个都抓完了
            else:
                return False  # 还没抓完, 继续

        return False

    # ==========================================================================
    #                     机械臂卸载物块(卸载区)
    # ==========================================================================
    def unload_block(self, cx=0, cy=0, cz=0):
        """
        在卸载区放下当前应该卸载的物块
        根据self.unload_order和self.unload_count确定放哪个
        返回: True=本次卸载完成
        """
        self.robot_move_cmd.car_move(0, 0, 0, 0)  # 确保停车

        current_block = self.unload_order[self.unload_count]

        # 发送卸货信息给STM32显示
        self.uart.write("$Unload:{}!\n".format(current_block))

        print("[UNLOAD] Unloading block:", current_block,
              "at zone", self.unload_count + 1)

        # 从码垛位置取出物块(后进先出, 最上面的先取)
        # 计算物块在码垛中的位置
        # 我们需要找到这个物块在grabbed_blocks中的索引
        block_index = -1
        for i, b in enumerate(self.grabbed_blocks):
            if b == current_block:
                block_index = i
                break

        if block_index == -1:
            print("[ERROR] Block not found in grabbed list!")
            self.unload_count += 1
            return True

        # 从码垛取物块
        pick_z = self.STACK_Z + block_index * 30

        # 移到码垛上方
        self.kinematic.kinematics_move(self.STACK_X, self.STACK_Y, 70, 1000)
        time.sleep_ms(1200)

        # 张开爪子
        self.kinematic.send_str("{#005P1000T1000!}")
        time.sleep_ms(500)

        # 下降到物块位置
        self.kinematic.kinematics_move(self.STACK_X, self.STACK_Y, pick_z, 1000)
        time.sleep_ms(1200)

        # 闭合抓取
        self.kinematic.send_str("{#005P1700T1000!}")
        time.sleep_ms(1200)

        # 抬起
        self.kinematic.kinematics_move(self.STACK_X, self.STACK_Y, 70, 1000)
        time.sleep_ms(1200)

        # 转向卸载区方向(机械臂伸到侧面)
        self.kinematic.kinematics_move(self.UNLOAD_X, self.UNLOAD_Y, 70, 1000)
        time.sleep_ms(1200)

        # 下降放置
        self.kinematic.kinematics_move(self.UNLOAD_X, self.UNLOAD_Y, -30 + cz, 1000)
        time.sleep_ms(1200)

        # 松开爪子
        self.kinematic.send_str("{#005P1000T1000!}")
        time.sleep_ms(1000)

        # 抬起回位
        self.kinematic.kinematics_move(self.UNLOAD_X, self.UNLOAD_Y, 70, 1000)
        time.sleep_ms(1200)

        # 回到巡线姿态
        self.kinematic.kinematics_move(self.LINE_FOLLOW_X, self.LINE_FOLLOW_Y,
                                        self.LINE_FOLLOW_Z, 1000)
        time.sleep_ms(1200)

        self.unload_count += 1
        print("[UNLOAD] Done. Total unloaded:", self.unload_count)

        return True

    # ==========================================================================
    #                           检查串口指令
    # ==========================================================================
    def check_uart_cmd(self):
        """
        检查STM32发来的串口指令
        支持的指令:
          #Order:ABC!  — 设置卸载顺序为A→B→C
          #Order:BAC!  — 设置卸载顺序为B→A→C
          #Order:...!  — 任意排列
          #Go!         — 启动任务
        """
        if self.uart.any():
            try:
                data = self.uart.read()
                if data:
                    string = data.decode()

                    # 解析卸载顺序指令: #Order:ABC!
                    idx = string.find("#Order:")
                    if idx >= 0:
                        end_idx = string.find("!", idx)
                        if end_idx >= 0:
                            order_str = string[idx + 7:end_idx]
                            if len(order_str) == 3:
                                self.unload_order = list(order_str)
                                print("[CMD] Unload order set to:",
                                      self.unload_order)

                    # 解析启动指令: #Go!
                    if string.find("#Go!") >= 0:
                        self.task_state = 1
                        print("[CMD] Task started! Order:",
                              self.unload_order)

            except Exception as e:
                print('[UART Error]:', e)

    # ==========================================================================
    #                           主运行循环
    # ==========================================================================
    def run(self, cx=0, cy=0, cz=0):
        """
        主循环调用此函数, 根据task_state执行不同阶段的任务
        cx/cy/cz: 机械臂抓取偏移微调参数
            cx: 偏右减小, 偏左增加
            cy: 偏前减小, 偏后增加
            cz: 偏高减小, 偏低增加

        任务流程(逆时针):
          状态0: 等待STM32发#Go!启动
          状态1: 巡线前往仓库区(检测到第一个红框=仓库区)
          状态2: 在仓库区抓取A/B/C三个物块
          状态3: 巡线前往卸载区(每到一个红框就停下卸货)
          状态4: 卸货
          状态5: 巡线返回启停区
          状态6: 任务完成, 停车
        """

        # ---- 始终检查串口指令 ----
        self.check_uart_cmd()

        # ================ 状态0: 等待启动 ================
        if self.task_state == 0:
            # 等待STM32发送#Go!指令
            time.sleep_ms(50)
            return

        # ================ 状态1: 巡线去仓库区 ================
        elif self.task_state == 1:
            img = sensor.snapshot()
            # 先用RGB检测红色边框
            has_red = self.detect_red_frame(img)

            if has_red and self.crossing_cnt == 0:
                # 检测到第一个红框 → 这是仓库区!
                # 小车前进一小段进入仓库区
                self.robot_move_cmd.car_move(self.BASE_SPEED, self.BASE_SPEED,
                                              self.BASE_SPEED, self.BASE_SPEED)
                time.sleep_ms(800)
                self.robot_move_cmd.car_move(0, 0, 0, 0)
                time.sleep_ms(300)

                self.crossing_cnt += 1
                print("[STATE1] Arrived at WAREHOUSE")

                # 切换机械臂到抓取姿态
                self.move_x = self.GRAB_X
                self.move_y = self.GRAB_Y
                self.kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep_ms(1500)

                # 关闭自动增益/白平衡, 提高颜色识别准确度
                sensor.set_auto_gain(False)
                sensor.set_auto_whitebal(False)
                time.sleep_ms(200)

                self.arm_state = 0
                self.mid_block_cnt = 0
                self.task_state = 2  # 进入仓库装货状态
                return

            # 没检测到红框, 继续巡线
            img_gray = img.to_grayscale()
            self.line_follow(img_gray)

        # ================ 状态2: 仓库区装货 ================
        elif self.task_state == 2:
            img = sensor.snapshot()
            done = self.grab_block(img, cx, cy, cz)

            if done:
                print("[STATE2] All 3 blocks grabbed!")

                # 恢复自动增益/白平衡
                sensor.set_auto_gain(True)
                sensor.set_auto_whitebal(True)

                # 机械臂回到巡线姿态
                self.kinematic.kinematics_move(self.LINE_FOLLOW_X,
                                                self.LINE_FOLLOW_Y,
                                                self.LINE_FOLLOW_Z, 1000)
                time.sleep_ms(1500)

                self.crossing_flag = 0
                self.red_frame_flag = 0
                self.task_state = 3  # 进入巡线去卸载区
                print("[STATE2→3] Start heading to unload zones")

        # ================ 状态3: 巡线去卸载区 ================
        elif self.task_state == 3:
            img = sensor.snapshot()

            # 检测红色边框(卸载区)
            has_red = self.detect_red_frame(img)

            if has_red and self.red_frame_flag == 0:
                self.red_frame_flag = 1  # 标记检测到红框

            if self.red_frame_flag == 1:
                # 再前进一小段让小车停在卸载区正前方
                self.robot_move_cmd.car_move(self.BASE_SPEED * 0.5,
                                              self.BASE_SPEED * 0.5,
                                              self.BASE_SPEED * 0.5,
                                              self.BASE_SPEED * 0.5)
                time.sleep_ms(500)
                self.robot_move_cmd.car_move(0, 0, 0, 0)
                time.sleep_ms(300)

                self.red_frame_flag = 0
                self.crossing_cnt += 1
                print("[STATE3] Arrived at UNLOAD zone", self.unload_count + 1)

                self.task_state = 4  # 进入卸货
                return

            # 没检测到红框, 继续巡线
            img_gray = img.to_grayscale()
            self.line_follow(img_gray)

        # ================ 状态4: 卸货 ================
        elif self.task_state == 4:
            done = self.unload_block(cx, cy, cz)

            if done:
                if self.unload_count >= 3:
                    # 三个都卸完了, 回启停区
                    print("[STATE4] All blocks unloaded! Heading home")
                    self.crossing_flag = 0
                    self.red_frame_flag = 0
                    self.task_state = 5
                else:
                    # 还有物块没卸, 继续巡线到下一个卸载区
                    print("[STATE4] Continue to next unload zone")
                    self.crossing_flag = 0
                    self.red_frame_flag = 0
                    self.task_state = 3

        # ================ 状态5: 巡线回启停区 ================
        elif self.task_state == 5:
            img = sensor.snapshot()

            # 检测红色边框(启停区)
            has_red = self.detect_red_frame(img)

            if has_red:
                # 前进进入启停区
                self.robot_move_cmd.car_move(self.BASE_SPEED * 0.5,
                                              self.BASE_SPEED * 0.5,
                                              self.BASE_SPEED * 0.5,
                                              self.BASE_SPEED * 0.5)
                time.sleep_ms(600)
                self.robot_move_cmd.car_move(0, 0, 0, 0)

                print("[STATE5] Arrived at START/STOP zone. DONE!")
                self.uart.write("$Done!\n")
                self.task_state = 6
                return

            # 继续巡线
            img_gray = img.to_grayscale()
            self.line_follow(img_gray)

        # ================ 状态6: 任务完成 ================
        elif self.task_state == 6:
            self.robot_move_cmd.car_move(0, 0, 0, 0)
            time.sleep_ms(100)
