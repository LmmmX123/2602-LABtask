import sensor
import image    #其实sensor已经隐式的引入了，但是还是留着吧
import time     #时间戳用的
from machine import UART    #硬件串口库，用于和外部设备通信
uart = UART(1, baudrate=115200, bits=8, parity=None, stop=1)

#---核心参数---
REAL_WID = 5.0        # 目标真实边长
FOCAL_LENGTH = 289      # 焦距（校过了）
BLACK_THRESH = (0, 18)  # 黑色灰度阈值
MIN_AREA = 200          # 最小色块面积（筛选目标）
WHITE_FIND = (230,255)  #后有反选所以在找区块的时候要搞一个白的
SAMPLE_RAN = 5      #采样次数
#INTER_TIME = 1      #采样间隔（已注释未启用）
OFFSET = 10  # 向内偏移的像素数，可根据实际情况调整


#--摄像头初始化--
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # 开灰度
sensor.set_framesize(sensor.QVGA)   #分辨率320x240
sensor.skip_frames(time=500)    #跳过开头0.5秒
sensor.set_auto_gain(False)    # 关闭自动增益，固定阈值生效
sensor.set_auto_whitebal(False) # 关闭白平衡
#clock = time.clock() 计算帧率可以加上


#---测距函数---
def get_distance(pixel_w):
    """相似三角形测距原理:
        D = (真实 * 焦距) / 像素"""
    if pixel_w < 1:     # 过滤无效像素宽度（避免除以0）
        return 0.0
    dist = (REAL_WID * FOCAL_LENGTH) / pixel_w    #核心公式
    return round(dist, 2)        # 保留2位小数，返回距离


#---单次测距函数---
def detect_once():
    img = sensor.snapshot() #取一帧图像
    img_binary = img.binary([BLACK_THRESH], invert=True)#灰度二值化+反选
    blobs = img_binary.find_blobs([WHITE_FIND], area_threshold=MIN_AREA, invert=True, merge=True)
     # 用来找目标色块：白色阈值、面积≥MIN_AREA、反选、合并相邻色块
    if blobs:
        target = max(blobs, key=lambda b: b.area())     # 选面积最大的色块
        x, y, w, h = target.rect()      # 获取目标的坐标（x,y）、宽度w、高度h
        ratio = min(w, h) / max(w, h)       #正方形长宽比为1，但是好像没啥用
        if 0.8 < ratio < 1.2:       #找到接近正方形的色块
            return get_distance(w)      # 用宽度w计算距离并返回
    return 0.0      #没有的就返回无效

#---功能二的实现，最后输出ERROR值---
def detect_error():
    """
    核心检测函数：持续检测色块，计算左右高度差（ERROR）
    返回值：tuple (H_left, H_right, ERROR) 或 None（未检测到目标）
    """
    img = sensor.snapshot()     # 采集一帧图
    img_binary = img.binary([BLACK_THRESH])     #二值化
    blobs = img_binary.find_blobs([(255, 255)],area_threshold=MIN_AREA,merge=True)
    # 找白色色块（即黑色目标），面积≥MIN_AREA，合并相邻色块

    if blobs:
        blob = max(blobs, key=lambda x: x.area())     #遍历所有色块找到最大的
        x0, y0, w, h = blob.rect()      #获取目标所有的有效信息，包含左上角坐标和宽和高

        # 计算需要偏移后的左右列坐标
        left_col = x0 + OFFSET
        right_col = x0 + w - 1 - OFFSET   #因为从0开始计数所以多减一个

        # 确保偏移后的列仍在色块范围内
        if left_col >= x0 + w or right_col < x0:
            print("偏移过大，超出色块范围")

        # 统计左列的具体高度
        left_col_pixels = []
        for cy in range(y0, y0 + h):        #遍历每一行
            if img_binary.get_pixel(left_col, cy) == 255:   
            #就是固定列的X，每一个图像中的固定列的每一个对应行元素
                left_col_pixels.append(cy)#形成具体的列的元素集

        # 统计右列的具体高度（同上）
        right_col_pixels = []
        for cy in range(y0, y0 + h):
            if img_binary.get_pixel(right_col, cy) == 255:
                right_col_pixels.append(cy)

        # 计算高度
        if left_col_pixels:
            H_left = max(left_col_pixels) - min(left_col_pixels)    #其实就是看有几个元素/像素
        else:
            H_left = h

        if right_col_pixels:
            H_right = max(right_col_pixels) - min(right_col_pixels)
        else:
            H_right = h

        ERROR = H_left - H_right        #核心要的ERROR值
        print((f"ERROR:{ERROR}\n"))     #调试输出
        uart.write(f"ERROR:{ERROR}\n")      #功能输出
        #uart.write(ERROR)
        time.sleep(0.2)
    else:
        print("未检测到目标")
        uart.write("ERROR:NONE\n")
        time.sleep(1)


#-----主循环-----
while True:
    if uart.any():
        data_byte = uart.read(1)        # 读取1个字节
        if data_byte is None:  # 只加这1行，过滤空数据
               continue
        print("收到字节:", data_byte)       #调试用的
        """
        入口一：功能一由此进入
        """
        if(data_byte == b'0'):
            dis_list = []       #取n次的数据，作为样本集做一个储存
            #---核心逻辑---
            for i in range(SAMPLE_RAN):     #采样5次（SAMPLE_RAN可改）
                d = detect_once()       # 单次测距
                if d > 0:  # 只加入有效距离
                    #print(a,d)
                    dis_list.append(d)      #加入列表
                #time.sleep(INTER_TIME)  # 采样间隔
            timestamp = time.ticks_ms()     #加入时间戳
            if dis_list:        #如果有效生成平均值
                avg_dist = sum(dis_list) / len(dis_list)        #平均值
                # 串口发送格式：时间戳 + 平均距离（保留2位小数）
                uart.write(f"timestamps: {timestamp}, result: {round(avg_dist, 2)}\n")
                print(f"timestamps: {timestamp}, result: {round(avg_dist, 2)}")

            else:       # 无有效数据 → 发送NONE
                uart.write(f"timestamps: {timestamp}, result: NONE\n")
                print(f"timestamps: {timestamp}, result: NONE")  # 无有效检测
        #else(data_byte !=)
            """
        入口二：功能二由此进入
         """
        elif(data_byte == b'1'):        #入口二
            while(1):       # 持续检测，直到收到停止指令
                if uart.any():      # 读取1个字节---主要是为了保障ctrl+C的中断
                    stop_byte = uart.read(1)
                    if stop_byte == b'\x00':
                        print("收到停止指令，退出标靶对正模式")
                        break  # 跳出持续循环，返回主循环
                detect_error()      #功能二执行
