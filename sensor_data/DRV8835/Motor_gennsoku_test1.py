
#モジュールインポート
import RPi.GPIO as GPIO
import time
from time import sleep 
import sys

#PIN指定
AIN1 = 15
AIN2 = 29


#GPIOのモード
GPIO.setmode(GPIO.BOARD)#物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)#←ここでエラーoutがだめ？→out×,,OUT〇
GPIO.setup(AIN2,GPIO.OUT)
#周波数設定
a1 = GPIO.PWM(AIN1,255)#255Hz
a2 = GPIO.PWM(AIN2,255)#255Hz

#PWM起動
a1.start(0)#Aenable接続（E）
a2.start(0)#Aphase接続（P）

duty = 100 #duty比　回転速度変更用変数
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


#ー－－－モーター減速プログラムー－－－－－

duty = 100                  #モーターの回転数値[0,100]
for sec in range(0,20,1):#０．１秒刻みのタイマー
    sec += 1
    duty -= 5
    print(duty)
    sleep(1)



