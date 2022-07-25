from lzma import PRESET_DEFAULT
import time
from micropyGPS import MicropyGPS
import pigpio
from math import *
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np
import csv

goal_latitude = 0
goal_longitude  =  0
radius = 6378.137
long_range_checked = False
short_range_checked = False
EPSIRON = 15
MAX_ULTRA_DISTANCE = 400
MIN_ULTRA_DISTANCE = 2
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

lat = 0
lon = 0

global pre_lat
global pre_lon
pre_lat = 0
pre_lon = 0

goal = False

global previous_ultra_distance
global current_ultra_distance

# 10秒ごとに表示
tm_last = 0
cnt = 0



#-------------モーター関係----------------------------------------------------------

AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33

#GPIOのモード
GPIO.setmode(GPIO.BOARD)    #物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)   #AIN1番ピンで
GPIO.setup(AIN2,GPIO.OUT)   #AIN2ピンで
GPIO.setup(BIN1,GPIO.OUT)   #BIN1ピンで
GPIO.setup(AIN2,GPIO.OUT)   #BIN2ピンでGPIO出力設定

#周波数設定
Apwm = GPIO.PWM(AIN2,255)     #PWM(pin,Hz)→AIN2番ピンで255Hz
Bpwm = GPIO.PWM(BIN2,255)     #PWM(pin,Hz)→BIN2番ピンで255Hz出力設定

def forward():        #前進
    #右車輪(A1,A2)=(+,0)→正回転
    Apwm.start(0)               #start(duty比)つまり出力0の状態              
    GPIO.output(AIN1,GPIO.HIGH) #AIN1を出力(pin,出力を100％) 
    Apwm.ChangeDutyCycle(0)     #ChangeDutyCycle(duty)→AIN2のduty比０に設定
    #左車輪(BIN1.BIN2)=(+,0)→正回転
    Bpwm.start(0)               
    GPIO.output(BIN1,GPIO.HIGH) #def forward()右車輪と同上
    Bpwm.ChangeDutyCycle(0)     
    
def turn_right():    #右回転
    #右車輪(AIN1,AIN2)=(0,+)→負回転
    Apwm.start(0)                
    GPIO.output(AIN1,GPIO.LOW)  #def forward()右車輪と同上
    Apwm.ChangeDutyCycle(100)   #duty比100％→255Hz（max）出力
    #左車輪(BIN1.BIN2)=(+,0)→正回転
    Bpwm.start(0)                
    GPIO.output(BIN1,GPIO.HIGH) #def forward()右車輪と同上
    Bpwm.ChangeDutyCycle(0)     
    
def turn_left():     #左回転
    #右車輪(AIN1.AIN2)=(+,0)→正回転
    Apwm.start(0)               
    GPIO.output(AIN1,GPIO.HIGH) #def forward()右車輪と同上
    Apwm.ChangeDutyCycle(0)     
    #左車輪(BIN1.BIN2)=(0,+)→負回転
    Bpwm.start(0)               
    GPIO.output(BIN1,GPIO.LOW)  #def forward()右車輪と同上
    Bpwm.ChangeDutyCycle(100)   

def back():          #後進
    #左車輪(AIN1.AIN2)=(0,+)→負回転
    Apwm.start(0)                  #def forward()右車輪と同上
    GPIO.output(AIN1,GPIO.LOW)
    Apwm.ChangeDutyCycle(75)
    #左車輪(BIN1.BIN2)=(+,0)→負回転
    Bpwm.start(0)                  #def forward()右車輪と同上
    GPIO.output(BIN1,GPIO.LOW)
    Bpwm.ChangeDutyCycle(100)
    
def stop():           #停止
    #左車輪(AIN1.AIN2)=(0,0)→停止
    Apwm.start(0)                  #def forward（）右車輪と同上
    GPIO.output(AIN1,GPIO.LOW)
    Apwm.ChangeDutyCycle(0)
    #左車輪(BIN1.BIN2)=(0,0)→停止
    Bpwm.start(0)                  #def forward()右車輪と同上
    GPIO.output(BIN1,GPIO.LOW)
    Bpwm.ChangeDutyCycle(0)

def azimuth(a,b,ap,bp):
    #a = lat, b = lon, ap = latp ,bp = lonp
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

def reading_ultrasound_distance(sensor):
    
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
        ultrasound_distance = timepassed * 17000
        return ultrasound_distance
        GPIO.cleanup()
    else:
        print ("Incorrect usonic() function varible.")


def red_detect(img):
    # HSV色空間に変換(ここでは、取得した映像を処理するために、RGBからHSVに変更するのと、赤色と認識するHSVの値域を設定してます。)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1(OpenCVのHSVの範囲は普通の範囲と違うので、ここではOpenCVで赤とされる0～30 150～179が色相としてます）[色相,彩度,明度]
    hsv_min = np.array([0, 127, 0])
    hsv_max = np.array([30, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max) #inRangeは映像を2値化する関数です(二値化するのはカラーより情報量が3分の1だからです)

    # 赤色のHSVの値域2
    hsv_min = np.array([150, 127, 0])
    hsv_max = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

    return mask1 + mask2

# ブロブ解析(二値化処理後の画像の中に表示されるものの形状を解析)

#　ブロブ解析(二値化処理後の画像の中に表示されるものの形状を解析)
def analysis_blob(binary_img):
    #違う関数で使いたいから、グローバルに。
    global label
    global data
    global max_index
    global center
    # 2値画像のラベリング処理(ラベリング処理は、認識した物体の輪郭を番号づけする処理。することで物体の形を認識できる)
    #connectedComponentsWithStatsはオブジェクトのサイズや重心などの情報も合わせて返してくれる関数
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
                maxblob["area"] = data[:, 4][max_index]   # 面積   面積は認識した物体ののピクセル数 / 全体のピクセル数で割り出される
                maxblob["center"] = center[max_index]  # 中心座標

                return maxblob #取得したブロブの情報を返す

    
    raise ValueError


with open('control_history.csv', 'w') as f_his, open('data_log.csv','w') as f_data:
    history = csv.writer(f_his)
    CanSat_log_data = csv.writer(f_data)

    time_row = my_gps.timestamp
    data_row = [time_row[0], time_row[1], time_row[2], my_gps.latitude[0], my_gps.longtitude[0], my_gps.altitude]

    CanSat_log_data.writerow(data_row)#センサーデータをdata_rowに全部格納して保存する
    while True:

        

        if not long_range_checked:#long rangeで一回のみ行う処理
            history.writerow('long range')

            long_range_checked = True

        (cnt, sentence) = serialpi.bb_serial_read(RX)
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
                            #ここまでsampleのコピペ

                            #ここから自作
                            lat = my_gps.latitude[0]
                            lon = my_gps.longitude[0]
                            #方位角計算
                            delta_fai = azimuth(lat,lon,pre_lat,pre_lon)
                            #距離計算
                            d = distance(lat,lon,pre_lat,pre_lon)
                            #モーター回転
                            motor(delta_fai,d)
                            history.writerow('rotating')
                            pre_lat = lat
                            pre_lon = lon

                            time.sleep(1)

        #近距離フェーズ
        if MIN_ULTRA_DISTANCE < reading_ultrasound_distance(0) and reading_ultrasound_distance(0) < MAX_ULTRA_DISTANCE or distance(lat,lon,pre_lat,pre_lon) < 10:
            

            
            if not short_range_checked:#short rangeで一回のみ行う処理
                history.writerow('short range')


                
                cap = cv2.VideoCapture(0)

                fps = 30

                # 録画する動画のフレームサイズ（webカメラと同じにする）
                size = (640, 480)

                # 出力する動画ファイルの設定（映像を取得して処理するスピードとfpsがかみ合わないと、保存した映像が早送りになってしまうのですが、安いラズパイ用のカメラでは12fpsくらいじゃないと早送りになってしまいました)
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                video = cv2.VideoWriter('CameraLog.avi', fourcc, fps, size)
                

                short_range_checked = True


            
            

            if cap.isOpened():
                # フレームを取得
                ret, frame = cap.read() #capに入れられた、カメラの映像を入れる

                # 赤色検出
                mask = red_detect(frame) #赤色検出の関数に読み込んだカメラの映像のデータを入れて処理

                #物体が検出されないと、次元数が2未満になり、ValueErrorが起きるので、起きたらフレームを飛ばす
                try:

                    # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
                    target = analysis_blob(mask)

                    # 面積最大ブロブの中心座標を取得
                    center_x = int(target["center"][0])
                    center_y = int(target["center"][1])

                    # フレームに面積最大ブロブの中心周囲に円を描く
                    cv2.circle(frame, (center_x, center_y), 50, (0, 200, 0),
                            thickness=3, lineType=cv2.LINE_AA)

                    
                    print("menseki",data[:, 4][max_index])
                    print(center[max_index][0])
                    # window表示
                    cv2.imshow("Frame", frame)
                    cv2.imshow("Mask", mask)

                    # 動画ファイルの書き込み
                    video.write(frame)
                
                except ValueError:
                    continue
                
                #モーター制御挿入
                #カメラの映像は横640にしてるので、320付近がカメラの中心です
                if  270 <= center[max_index][0] and center[max_index][0] < 370 and 50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
                    #まっすぐ進み動作(物体が中心近くにいる際)
                    forward()
                    history.writerow('progressing')
                
                elif data[:, 4][max_index] < 50:
                    #回転動作("赤い物体を検出できなくなった際")
                    turn_right()
                    history.writerow('searching for the goal')

                elif center[max_index][0] < 270 and 50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
                    #回転する動作(物体がカメラの中心から左にずれている際)
                    turn_left()
                    history.writerow('rotating right')
                
                elif center[max_index][0] >= 370 and 50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
                    #回転する動作（物体がカメラの中心から右にずれている際）
                    turn_right()
                    history.writerow('rotating left')

                elif data[:, 4][max_index] > 80000:
                    #止まる(物体の近くに接近した際)
                    stop()
                    history.writerow('stopped')

                

                # qキーが押されたら途中終了(何かあった時ように、手動停止)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break

            cap.release()
            video.release()
            cv2.destroyAllWindows()

            while True:#距離センサを使ったゴールサーチと判定(仮)
                search_count = 0
                history.writerow('searching for the goal')
                if MIN_ULTRA_DISTANCE < reading_ultrasound_distance(0) and reading_ultrasound_distance(0) < MAX_ULTRA_DISTANCE:
                    stop()#停止
                    history.writerow('stopped')
                    if reading_ultrasound_distance(0) < EPSIRON:
                        print("goal")
                        history.writerow('GOAL!')
                        goal = True
                    break
                else:
                    turn_left()
                    search_count += 1
                    if search_count > 5:
                        break
                
            if goal:
                break
            time_row = my_gps.timestamp
            data_row = [time_row[0], time_row[1], time_row[2], my_gps.latitude[0], my_gps.longtitude[0], my_gps.altitude]

            CanSat_log_data.writerow(data_row)#センサーデータをdata_rowに全部格納して保存する
                
