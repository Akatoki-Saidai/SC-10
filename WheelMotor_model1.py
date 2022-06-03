
#モジュールインポート
import RPi.GPIO as GPIO
import time 
import sys

#PIN指定
AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33

#GPIOのモード
GPIO.setmode(GPIO.BOARD)#物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)#←ここでエラーoutがだめ？→Result:out×,,OUT〇
GPIO.setup(AIN2,GPIO.OUT)

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

duty = 25 #duty比　回転速度変更用変数
#DDRV8355 = MODE0 

#右モータ関数
def right_forward():#E=1 P=0の時
    a1.ChangeDutyCycle(duty)
    a2.ChangeDutyCycle(0)


def right_back():#E=1 P=1の時
    a1.ChangeDutyCycle(0)
    a2.ChangeDutyCycle(duty)

def right_stop():#E=0 P=0?の時
    a1.ChangeDutyCycle(0)
    a2.ChangeDutyCycle(0)
    
#左モーター関数
def left_forward():
    b1.ChangeDutyCycle(duty)
    b2.ChangeDutyCycle(0)

def left_back():
    b1.ChangeDutyCycle(0)
    b2.ChangeDutyCycle(duty)

def left_stop():
    b1.ChangeDutyCycle(0)
    b2.ChangeDutyCycle(0)

while True:
    x = int(input('kaiten'))#→考え：何回かのデータから平均をとるべき→外れ値の除外目的

    if 200 < x < 400:#前進
        right_forward()
        left_forward()
    elif 0 < x <= 200:#右回転
        right_stop()
        left_forward()

    elif 400 <= x < 600:#左回転
        right_forward()
        left_stop()

#タイヤの後進いる？？？？回転はどうすべきか（　＾ω＾）・・・
        
#時々終了してもストップしないときがある






