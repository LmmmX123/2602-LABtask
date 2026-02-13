import sensor
import image    #其实sensor已经隐式的引入了，但是还是留着吧
import time
from machine import UART
uart = UART(1, baudrate=115200, bits=8, parity=None, stop=1)

#---核心参数---
REAL_WID = 5.0        # 目标真实边长
FOCAL_LENGTH = 289      # 焦距（校过了）
BLACK_THRESH = (0, 10)  # 黑色灰度阈值
MIN_AREA = 200          # 最小色块面积
WHITE_FIND = (230,255)  #后有反选所以在找区块的时候要搞一个白的
SAMPLE_RAN = 5
INTER_TIME = 1
#--摄像头初始化--
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # 开灰度
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=500)    #跳过开头0.1秒
sensor.set_auto_gain(False)    # 关闭自动增益，固定阈值生效
sensor.set_auto_whitebal(False) # 关闭白平衡
#clock = time.clock() 计算帧率可以加上

#---测距函数---
def get_distance(pixel_w):
    """相似三角形测距原理:
        D = (真实 * 焦距) / 像素"""
    if pixel_w < 1:
        return 0.0
    dist = (REAL_WID * FOCAL_LENGTH) / pixel_w
    return round(dist, 2)

#---单次测距函数---
def detect_once():
    img = sensor.snapshot() #取图像
    img_binary = img.binary([BLACK_THRESH], invert=True)#灰度处理
    blobs = img_binary.find_blobs([WHITE_FIND], area_threshold=MIN_AREA, invert=True, merge=True)
    if blobs:
        target = max(blobs, key=lambda b: b.area())
        x, y, w, h = target.rect()
        ratio = min(w, h) / max(w, h)#正方形长宽比为1，但是好像没啥用
        if 0.8 < ratio < 1.2:
            return get_distance(w)
    return 0.0

#---计算并打印平均值---
while True:
    if uart.any():
        # 读取1个字节
        data_byte = uart.read(1)
        print("收到字节:", data_byte)
        if(data_byte == b'1'):
            dis_list = []
            #---核心逻辑---
            a = 1
            for i in range(SAMPLE_RAN):
                d = detect_once()
                if d > 0:  # 只加入有效距离
                    #print(a,d)
                    a += 1
                    dis_list.append(d)
                #time.sleep(INTER_TIME)  # 采样间隔
            timestamp = time.ticks_ms()
            if dis_list:
                avg_dist = sum(dis_list) / len(dis_list)
                uart.write(f"timestamps: {timestamp}, result: {round(avg_dist, 2)}\n")
                print(f"timestamps: {timestamp}, result: {round(avg_dist, 2)}")
            else:
                uart.write(f"timestamps: {timestamp}, result: NONE\n")
                print(f"timestamps: {timestamp}, result: NONE")  # 无有效检测
        #else(data_byte !=)
