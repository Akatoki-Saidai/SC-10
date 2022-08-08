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

goal_latitude = 0
goal_longitude  =  0
radius = 6378.137

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
            if(losting == 1):
                CW()
                time.sleep(0.3)
                stop()
                time.sleep(2)
                if(losting == 0):
                    break
        
@atexit.register  
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
    

def red_detect(img):
    # HSV色空間に変換
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0, 180, 50])
    hsv_max = np.array([15, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([165, 180, 50])
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
                                         data[:, 1][max_index])  # 左上座標
                maxblob["width"] = data[:, 2][max_index]  # 幅
                maxblob["height"] = data[:, 3][max_index]  # 高さ
                maxblob["area"] = data[:, 4][max_index]   # 面積
                maxblob["center"] = center[max_index]  # 中心座標

                return maxblob

    
    raise ValueError




def main():
   
    # カメラのキャプチャ
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
     
    fps = 15

    # 録画する動画のフレームサイズ（webカメラと同じにする）
    size = (640, 480)

    #見しうなった時のキャプチャー
    global losting

    # 出力する動画ファイルの設定
    fourcc = cv2.VideoWriter_fourcc(*'H264')
    video = cv2.VideoWriter('VIDEO.avi', fourcc, fps, size)

    baudrate = 9600

    TX = 24
    RX = 23

    serialpi = pigpio.pi()
    serialpi.set_mode(RX,pigpio.INPUT)
    serialpi.set_mode(TX,pigpio.OUTPUT)

    pigpio.exceptions = False
    serialpi.bb_serial_read_close(RX)
    pigpio.exceptions = True

    serialpi.bb_serial_read_open(RX,baudrate,8)
    # gps設定
    my_gps = MicropyGPS(9, 'dd')

    lat  = 0
    long = 0 

    global pre_lat
    global pre_long
    pre_lat = 0
    pre_long = 0

    # 10秒ごとに表示
    tm_last = 0
    count = 0

    with open("log.csv","w") as f:
        writer = csv.writer(f)
        while(cap.isOpened()):
            # フレームを取得
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            _, frame = cap.read()

            

            #映像反転
            frame = cv2.rotate(frame,cv2.ROTATE_180)
            
            # 赤色検出
            mask = red_detect(frame)

            (count, sentence) = serialpi.bb_serial_read(RX)
            if len(sentence) > 0:
                for x in sentence:
                    if 10 <= x <= 126:
                        stat = my_gps.update(chr(x))
                        if stat:
                            tm = my_gps.timestamp
                            tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                            if (tm_now - tm_last) >= 10:
                                print('=' * 20)
                                print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
                                print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0])
                                row = [(my_gps.latitude[0], my_gps.longitude[0])]
                                writer.writerow(row)
                                #ここまでsampleのコピペ

                                #ここから自作
                                lat = my_gps.latitude[0]
                                long = my_gps.longitude[0]
                                #方位角計算
                                delta_fai = azimuth(lat,long,pre_lat,pre_long)
                                #距離計算
                                d = distance(lat,long,pre_lat,pre_long)
                                #モーター回転
                                motor(delta_fai,d)
                                pre_lat = lat
                                pre_long = long
            
            
            try:

                # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
                target = analysis_blob(mask)

                # 面積最大ブロブの中心座標を取得
                center_x = int(target["center"][0])
                center_y = int(target["center"][1])

                # フレームに面積最大ブロブの中心周囲を円で描く
                cv2.circle(frame, (center_x, center_y), 50, (0, 200, 0),
                           thickness=3, lineType=cv2.LINE_AA)
                #GPS(距離)
                d = str(distance(lat,long,pre_lat,pre_long))

                print("menseki",data[:, 4][max_index])
                #print("menseki",data[:, 4][max_index])
                #print(center[max_index][0])
             
                # 結果表示
                cv2.imshow("Frame", frame)
                cv2.imshow("Mask", mask)

                 # 書き込み
                video.write(frame)
            
            except ValueError:
                video.write(frame)
                losting = 1

            
            if cv2.waitKey(25) & 0xFF == ord('q'):
                GPIO.cleanup()
                break
        
        
        cap.release()
        video.release()
        cv2.destroyAllWindows()

def motor_processing():
    while True:
        try:
            dimensions = data.shape
            if dimensions:
        # 2次元以上であること。※data[:,4]より2次元目のindex=4を参照しているため
                if len(dimensions) >= 2:
            # 2次元目の要素数を確認
                    dim2nd = dimensions[1]
            # 2次元目の要素数5以上ならdata[:,4]の2次元目のindex=4の条件を満たす
                    if dim2nd >= 5:
                        losting = 0
                        if  170 <= center[max_index][0] and center[max_index][0] < 470 and  50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
                #まっすぐ進み動作(物体が中心近くにいる際)
                            forward()
                            time.sleep(1)
                            stop()
                            time.sleep(1)
        
                        elif data[:, 4][max_index] <= 50:
                            CW()
                            time.sleep(0.3)
                            stop()
                            time.sleep(2)
        

                        elif center[max_index][0] < 170 and  50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
                    #回転する動作(物体がカメラの中心から左にずれている際)
                            CCW()
                            time.sleep(1)
                            stop()
                            time.sleep(1)
        
                        elif center[max_index][0] >= 470 and  50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
                    #回転する動作（物体がカメラの中心から右にずれている際）
                            CW()
                            time.sleep(1)
                            stop()
                            time.sleep(1)

                        elif data[:, 4][max_index] > 80000:
                    #止まる(物体の近くに接近した際)
                            stop()

                            
        except IndexError:
            continue



                
                              
if __name__ == "__main__":
    thread_main = threading.Thread(target=main)
    thread_motor_processing = threading.Thread(target=motor_processing)
    thread_losting = threading.Thread(target=lost)
    thread_main.start()
    time.sleep(3)
    thread_losting.start()
    thread_motor_processing.start()
