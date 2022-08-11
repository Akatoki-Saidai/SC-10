from lzma import PRESET_DEFAULT
import time
from micropyGPS import MicropyGPS
import pigpio
from math import *

goal_latitude = 0
goal_longitude  =  0
radius = 6378.137


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
    d = r*acos(sin(ap)*sin(a)+cos(ap)*cos(a)*cos(b-bp))
    return d

def motor(fai,d):
    ##モーター回転プログラム
    ##Ⅰ秒で何度回る？？
    return 0



def main():
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
    while True:
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
                            
                            
                            
                            
    
    
if __name__ == "__main__":
    main()
    
##gpsのサンプルデータを基盤に作ったので後で整理する
