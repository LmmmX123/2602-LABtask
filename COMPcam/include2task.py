import cv2
import numpy as np
import time
import serial

# 你原来的代码原样，只换串口
uart = serial.Serial("COM6", 115200, timeout=0)

# --核心参数：按你现在的场景微调阈值--
REAL_WID = 5.0
FOCAL_LENGTH = 312
BLACK_THRESH = (30, 62)  
MIN_AREA = 50            
WHITE_FIND = (230,255)
SAMPLE_RAN = 5
OFFSET = 10

# ---新增：中间裁剪参数（仅加这几行）---
CROP_W = 200  # 裁剪后宽度，可根据需要微调（如180/220）
CROP_H = 200  # 裁剪后高度
# 计算裁剪起始坐标，保证裁在画面正中间（320×240分辨率）
crop_x_start = (320 - CROP_W) // 2
crop_y_start = (240 - CROP_H) // 2

# --摄像头初始化--
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
# 注释掉容易报错的灰度模式，手动转灰度更稳定
# cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)

# ---测距函数---
def get_distance(pixel_w):
    if pixel_w < 1:
        return 0.0
    dist = (REAL_WID * FOCAL_LENGTH) / pixel_w
    return round(dist, 2)

# ---单次测距函数：仅加中间裁剪，其余逻辑完全不动---
def detect_once():
    ret, frame = cap.read()
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("gray", img_gray)

    # ---新增：中间裁剪（仅加这几行）---
    img_crop = img_gray[
        crop_y_start : crop_y_start + CROP_H,
        crop_x_start : crop_x_start + CROP_W
    ]
    cv2.imshow("crop_gray", img_crop)  # 可视化裁剪后的灰度图

    # 3. 二值化：改为在裁剪后的图上做，其余不变
    _, img_binary = cv2.threshold(img_crop, BLACK_THRESH[1], 255, cv2.THRESH_BINARY_INV)
    # 4. 可视化二值图（确认纯白目标+纯黑背景）
    cv2.imshow("binary", img_binary)
    cv2.waitKey(1)

    contours, _ = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        blobs.append( (area, x, y, w, h) )

    if blobs:
        target = max(blobs, key=lambda b: b[0])
        area, x, y, w, h = target
        ratio = min(w, h) / max(w, h)
        if 0.8 < ratio < 1.2:  
            return get_distance(w)
    return 0.0

# ---功能二的实现：加了中间裁剪，其余完全不动---
def detect_error():
    ret, frame = cap.read()
    # 1. 强制转灰度
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ---新增：中间裁剪（仅加这几行）---
    img_crop = img_gray[
        crop_y_start : crop_y_start + CROP_H,
        crop_x_start : crop_x_start + CROP_W
    ]

    # 2. 二值化：改为在裁剪后的图上做，其余不变
    _, img_binary = cv2.threshold(img_crop, BLACK_THRESH[1], 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        blobs.append( (area, x, y, w, h) )

    if blobs:
        blob = max(blobs, key=lambda x: x[0])
        area, x0, y0, w, h = blob

        left_col = x0 + OFFSET
        right_col = x0 + w - 1 - OFFSET

        if left_col >= x0 + w or right_col < x0:
            print("偏移过大，超出色块范围")

        left_col_pixels = []
        for cy in range(y0, y0 + h):
            if img_binary[cy, left_col] == 255:
                left_col_pixels.append(cy)

        right_col_pixels = []
        for cy in range(y0, y0 + h):
            if img_binary[cy, right_col] == 255:
                right_col_pixels.append(cy)

        if left_col_pixels:
            H_left = max(left_col_pixels) - min(left_col_pixels)
        else:
            H_left = h

        if right_col_pixels:
            H_right = max(right_col_pixels) - min(right_col_pixels)
        else:
            H_right = h

        ERROR = (H_left - H_right)
        if abs(ERROR) > 1:
            ERROR = ERROR * 2  # 大于±2才放大2倍
        print(f"ERROR:{ERROR}\n")
        uart.write(f"ERROR:{ERROR}\n".encode())
        time.sleep(0.2)
    else:
        print("未检测到目标")
        uart.write(b"ERROR:NONE\n")
        time.sleep(1)

# -----主循环-----
while True:
    if uart.in_waiting > 0:
        data_byte = uart.read(1)
        if data_byte is None:
            continue
        print("收到字节:", data_byte)

        if(data_byte == b'0'):
            dis_list = []
            for i in range(SAMPLE_RAN):
                d = detect_once()
                if d > 0:
                    dis_list.append(d)
            timestamp = time.time()
            if dis_list:
                avg_dist = sum(dis_list) / len(dis_list)
                uart.write(f"timestamps: {timestamp}, result: {round(avg_dist, 2)}\n".encode())
                print(f"timestamps: {timestamp}, result: {round(avg_dist, 2)}")
            else:
                uart.write(f"timestamps: {timestamp}, result: NONE\n".encode())
                print(f"timestamps: {timestamp}, result: NONE")

        elif(data_byte == b'1'):
            while(1):
                if uart.in_waiting > 0:
                    stop_byte = uart.read(1)
                    if stop_byte == b'\x00':
                        print("收到停止指令，退出标靶对正模式")
                        break
                detect_error()

# 释放资源（规范写法，不影响原有逻辑）
cap.release()
cv2.destroyAllWindows()