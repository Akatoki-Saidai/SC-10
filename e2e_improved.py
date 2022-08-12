from lzma import PRESET_DEFAULT
import time
from micropyGPS import MicropyGPS
import pigpio
from math import *
import cv2
import numpy as np
import RPi.GPIO as GPIO #GPIOインポート
import time#時間制御インポート
import threading
import csv
import atexit
import smbus
import datetime
import sys
from Adafruit_BNO055 import BNO055
import logging

data = 0
max_index = 0
stop_last = 0

#PIN指定
AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33

#GPIOのモード
GPIO.setmode(GPIO.BOARD)#物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(AIN2,GPIO.OUT)
GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)
#周波数設定
a1 = GPIO.PWM(AIN1,255)#255Hz
a2 = GPIO.PWM(AIN2,255)#255Hz
b1 = GPIO.PWM(BIN1,255)
b2 = GPIO.PWM(BIN2,255)
#PWM起動
a1.start(0)#Aenable接続（E）
a2.start(0)#Aphase接続（P）
b1.start(0)
b2.start(0)

duty = 50 #duty比　回転速度変更用変数
#DDRV8355 = MODE0 
#--------------右モータ関数--------------
def right_forward():#E=1 P=0の時
    a1.ChangeDutyCycle(duty)
    a2.ChangeDutyCycle(0)


def right_back():#E=1 P=1の時
    a1.ChangeDutyCycle(0)
    a2.ChangeDutyCycle(duty)

def right_stop():#E=0 P=0?の時
    a1.ChangeDutyCycle(0)
    a2.ChangeDutyCycle(0)
#----------左モーター関数-----------
def left_forward():
    b1.ChangeDutyCycle(duty)
    b2.ChangeDutyCycle(0)

def left_back():
    b1.ChangeDutyCycle(0)
    b2.ChangeDutyCycle(duty)

def left_stop():
    b1.ChangeDutyCycle(0)
    b2.ChangeDutyCycle(0)



#-----------動く方向関数---------
def CCW():#前進
    right_forward()
    left_forward()

def back():#右回転
    right_back()
    left_forward()

def forward():#左回転
    right_forward()
    left_back()

def stop():#停止
    right_stop()
    left_stop()
    
def CW():#後進
  right_back()
  left_back()

def lost():
    while True:
        while True:
            if(losing == 1):
                CW()
                time.sleep(0.2)
                stop()
                time.sleep(3)
                time.sleep(2)
                if(losing == 0):
                    break
        
 
def CLEAN():
    GPIO.cleanup()
  
    
def azimuth(a,b,ap,bp):
    #a = lat, b = long, ap = latp ,bp = longp
    ag = goal_latitude
    bg = goal_longitude
    fai1  = atan2(sin(b - bp),cos(bp)*tan(b) - sin(bp)*cos(a - ap))
    fai2 =  atan2(sin(b - bg),cos(bg)*tan(b) - sin(bg)*cos(a - ag))
    dfai = fai2 - fai1 #回転する角度
    return dfai

def distance(a,b,ap,bp):
    r = radius
    d = r*acos(sin(ap)*sin(bp)+cos(bp)*cos(b)*cos(a-ap))
    return d

def motor(fai,d):
    ##モーター回転プログラム
    ##Ⅰ秒で何度回る？？
    return 0
#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
def reading(sensor):
    import time
    import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
     
    GPIO.setmode(GPIO.BOARD)
    TRIG = 11
    ECHO = 13
     
    if sensor == 0:
        GPIO.setup(TRIG,GPIO.OUT)
        GPIO.setup(ECHO,GPIO.IN)
        GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.3)
         
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
 
        while GPIO.input(ECHO) == 0:
          signaloff = time.time()
         
        while GPIO.input(ECHO) == 1:
          signalon = time.time()
 
        timepassed = signalon - signaloff
        distance = timepassed * 17000
        return distance
        GPIO.cleanup()
    else:
        print ("Incorrect usonic() function varible.")
    

def red_detect(img):
    # HSV色空間に変換
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0, 200, 190])
    hsv_max = np.array([15, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max) "inRange(hsvデータ ,　二値化する色の最小 , 二値化する色の最大)

    # 赤色のHSVの値域2(HSV空間は360で考えるから(OpenCVのhsvは(色相 , 彩度 , 明度)の範囲が(0～180 , 0～255 , 0～255))
    hsv_min = np.array([165, 200, 190])
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
    label = cv2.connectedComponentsWithStats(binary_img)　#このメソッドについては、https://axa.biopapyrus.jp/ia/opencv/object-detection.htmlを参考にするとわかりやすい

    # ブロブ情報を項目別に抽出
    n = label[0] - 1
    data = np.delete(label[2], 0, 0)　#np.delete(入力配列 , 削除する行番号や列番号 , 削除対象の軸)
    center = np.delete(label[3], 0, 0)

    # 配列の次元数を取得
    dimensions = data.shape

    # dimensionsが空ではない
    if dimensions:
        # 2次元以上であること。※data[:,4]より2次元目のindex=4を参照しているため
        if len(dimensions) >= 2:
            """
              axis =1
           a  [[0 1 2 3 4　　1次元
           x    5 6 7 8 9　　2次元
           i    10 11 12 13 14]]    3次元
           s
           =
           0
             """
            # 2次元目の要素数を確認
            dim2nd = dimensions[1]
            """
             [[0 1 2 3 4　　1次元  [0]
               5 6 7 8 9　　2次元  [1]
               10 11 12 13 14]]    3次元 [2]
            """
            # 2次元目の要素数5以上ならdata[:,4]の2次元目のindex=4の条件を満たす
            if dim2nd >= 5:  #例　[1](2次元の要素が5個(5 6 7 8 9)あるから、条件を満たす
                # ブロブ面積最大のインデックス
                max_index = np.argmax(data[:, 4])

                # 面積最大ブロブの情報格納用
                maxblob = {}

              # 面積最大ブロブの各種情報を取得
                maxblob["upper_left"] = (data[:, 0][max_index],
                                         data[:, 1][max_index])  # 左上座標
                maxblob["width"] = data[:, 2][max_index]  # 幅
                maxblob["height"] = data[:, 3][max_index]  # 高さ
                maxblob["area"] = data[:, 4][max_index]   # 面積
                maxblob["center"] = center[max_index]  # 中心座標

                return maxblob

    
    raise ValueError


def camera_stop():
    global stop_last
    stop_last = 0
    while True:
        try:
            if cv2.waitKey(25) & 0xFF == ord('q') or data[:, 4][max_index] > 80000:
                GPIO.cleanup()
                stop_last = 1
                
        except IndexError:
            CW()
            time.sleep(0.2)
            stop()
            time.sleep(3)
            time.sleep(2)

        
        
    cap.release()
    video.release()
    cv2.destroyAllWindows()



def main():
   
    # カメラのキャプチャ
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
     
    fps = 15

    # 録画する動画のフレームサイズ（webカメラと同じにする）
    size = (640, 480)

    #見しうなった時のキャプチャー
    global losing
    losing = 0

    # 出力する動画ファイルの設定
    fourcc = cv2.VideoWriter_fourcc(*'H264')#H264は圧縮フォーマット形式なので、速度を上昇させることを目的で)
    video = cv2.VideoWriter('VIDEO.avi', fourcc, fps, size)
    with open('area.csv', 'w') as fcap, open('hcsr04.csv', 'w') as fh: 
        area = csv.writer(fcap)
        area.writerow(['area','center'])
        hcsr04 = csv.writer(fh)
        hcsr04.writerow(['distance'])
        while(cap.isOpened()):#映像が読み込まれているのならば、 cap.isOpened() = True となり、ループが行われる
            # フレームを取得
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) #バッファサイズを１で指定することにより、現在の映像を取得
            _, frame = cap.read()　

            

            #映像反転
            frame = cv2.rotate(frame,cv2.ROTATE_180) #カメラを反対に搭載しているため.処理を減らしたいのであれば、後で反転されるほうがいい.
            
            # 赤色検出
            mask = red_detect(frame)
            
            try:　#赤い物体を認識できなくなったとき,ValueErrorが発生するので例外処理を行っている.

                # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
                target = analysis_blob(mask)

                # 面積最大ブロブの中心座標を取得
                center_x = int(target["center"][0])
                center_y = int(target["center"][1])

                # フレームに面積最大ブロブの中心周囲を円で描く
                cv2.circle(frame, (center_x, center_y), 50, (200, 0, 0), #(映像のデータ,円の中心,円の半径,円の色(青,緑,赤),円の厚さ,線を描写するアルゴリズム)
                           thickness=3, lineType=cv2.LINE_AA)


                print("menseki",data[:, 4][max_index])
                print("distance",reading(0))
                area_row = [data[:, 4][max_index],center_x]
                area.writerow(area_row)
                distance_row = [reading(0)]
                hcsr04.writerow(distance_row)
                #print("menseki",data[:, 4][max_index])
                #print(center[max_index][0])
             
                # 結果表示
                cv2.imshow("Frame", frame)
                cv2.imshow("Mask", mask)

                 # 書き込み
                video.write(frame)
            
            except ValueError:
                cv2.imshow("Frame", frame)
                cv2.imshow("Mask", mask)
                video.write(frame)
                losing = 1
                
            if stop_last == 1:
                break



def motor_processing():
    while True:
        try:
            dimensions = data.shape
            if dimensions:
        # 2次元以上であること。※data[:,4]より2次元目のindex=4を参照しているため
                if len(dimensions) >= 2:
                    losing = 0
            # 2次元目の要素数を確認
                    dim2nd = dimensions[1]
            # 2次元目の要素数5以上ならdata[:,4]の2次元目のindex=4の条件を満たす
                    if dim2nd >= 5:
                        if  150 <= center[max_index][0] and center[max_index][0] < 540 and  50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
                #まっすぐ進み動作(物体が中心近くにいる際)
                            forward()
                            time.sleep(3)
                            time.sleep(2)
                            CCW()
                            time.sleep(0.3)
                            stop()
                            time.sleep(1)
                        elif data[:, 4][max_index] <= 50 :
                            CW()
                            time.sleep(0.2)
                            stop()
                            time.sleep(2)
                            time.sleep(2)
                            
                        elif center[max_index][0] < 150 and  50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
                    #回転する動作(物体がカメラの中心から左にずれている際)
                            CCW()
                            time.sleep(0.2)
                            stop()
                            time.sleep(1)
        
                        elif center[max_index][0] >= 540 and  50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
                    #回転する動作（物体がカメラの中心から右にずれている際）
                            CW()
                            time.sleep(0.2)
                            stop()
                            time.sleep(1)

                        elif data[:, 4][max_index] > 80000:
                    #止まる(物体の近くに接近した際)
                            stop()
                            GPIO.cleanup()
                            break

                            
        except IndexError:
            continue



                
                              
if __name__ == "__main__":
    thread_main = threading.Thread(target=main)
    thread_motor_processing = threading.Thread(target=motor_processing)
    thread_losting = threading.Thread(target=lost)
    thread_camera_stop = threading.Thread(target=camera_stop)
    thread_main.start()
    time.sleep(3)
    time.sleep(1)
    thread_losting.start()
    thread_motor_processing.start()
    thread_camera_stop.start()
    atexit.register(CLEAN)
    
    
    
