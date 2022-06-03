# -*- coding: utf-8 -*-
import cv2
import numpy as np
import RPi.GPIO as GPIO
import sys

duty = 80

GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)

p1 = GPIO.PWM(27, 50) #50Hz
p2 = GPIO.PWM(22, 50) #50Hz
            
p1.start(0)
p2.start(0)


class UserException(ValueError):
    pass


def red_detect(img):
    # HSV色空間に変換
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0, 127, 0])
    hsv_max = np.array([30, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([150, 127, 0])
    hsv_max = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

    return mask1 + mask2

# ブロブ解析


def analysis_blob(binary_img):
    global label
    global data
    global max_index
    global center
    # 2値画像のラベリング処理
    label = cv2.connectedComponentsWithStats(binary_img)

    # ブロブ情報を項目別に抽出
    n = label[0] - 1
    data = np.delete(label[2], 0, 0)
    center = np.delete(label[3], 0, 0)

    # 配列の次元数を取得
    dimensions = data.shape

    # dimensionsが空ではない
    if dimensions:
        # 2次元以上であること。※data[:,4]より2次元目のindex=4を参照しているため
        if len(dimensions) >= 2:
            # 2次元目の要素数を確認
            dim2nd = dimensions[1]
            # 2次元目の要素数5以上ならdata[:,4]の2次元目のindex=4の条件を満たす
            if dim2nd >= 5:
                # ブロブ面積最大のインデックス
                max_index = np.argmax(data[:, 4])

                # 面積最大ブロブの情報格納用
                maxblob = {}

                # 面積最大ブロブの各種情報を取得
                maxblob["upper_left"] = (data[:, 0][max_index],
                                         data[:, 1][max_index])  # 左上を原点とした座標
                maxblob["width"] = data[:, 2][max_index]  # 幅
                maxblob["height"] = data[:, 3][max_index]  # 高さ
                maxblob["area"] = data[:, 4][max_index]   # 面積
                maxblob["center"] = center[max_index]  # 中心座標

                return maxblob

    
    raise ValueError


def main():
    videofile_path = "C:/github/sample/python/opencv/video/color_tracking/red_pendulum.mp4"

    # カメラのキャプチャ
    cap = cv2.VideoCapture(0)

    while(cap.isOpened()):
        # フレームを取得
        ret, frame = cap.read()

        # 赤色検出
        mask = red_detect(frame)

        
        try:

            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            target = analysis_blob(mask)

            # 面積最大ブロブの中心座標を取得
            center_x = int(target["center"][0])
            center_y = int(target["center"][1])

            # フレームに面積最大ブロブの中心周囲を円で描く
            cv2.circle(frame, (center_x, center_y), 50, (0, 200, 0),
                       thickness=3, lineType=cv2.LINE_AA)

            
            print("menseki",data[:, 4][max_index])
            print(center[max_index])
            if (data[:, 4][max_index]  <= 0):
               break
            # 結果表示
            cv2.imshow("Frame", frame)
            cv2.imshow("Mask", mask)
        
        except ValueError:
            continue

        motor()

def motor():
    while True:
       #遠くにいた場合全身
        c = sys.stdin.read(1)
        if data[:, 4][max_index] <30000: #
            p1.ChangeDutyCycle(duty)
            p2.ChangeDutyCycle(duty)
              
        #中心座標によって左に曲がる
        if int(target["center"][0]) < 200:
            p1.ChangeDutyCycle(0)
            p2.ChangeDutyCycle(duty)

          #中心座標によって左に曲がる
        if int(target["center"][0]) > 500:
            p1.ChangeDutyCycle(0)
            p2.ChangeDutyCycle(duty)

        #近づき具合によって終了
        if data[:, 4][max_index] >30000:
            p1.ChangeDutyCycle(0)
            p2.ChangeDutyCycle(0)
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
