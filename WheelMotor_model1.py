
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
GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN1,GPIO.out)
GPIO.setup(AIN2,GPIO.out)
#周波数設定
a1 = GPIO.PWM(AIN1,255)#255Hz
a2 = GPIO.PWM(AIN2,255)#255Hz

a1.start(25)#Aenable接続（E）
a2.start(25)#Aphase接続（P）

duty = 50 #duty比５０パーセント  

def right_forward():#E=1 P=0の時
    a1.ChangeDutyCycle(duty)
    a2.ChangeDutyCycle(0)


def right_back():#E=1 P=1の時
    a1.ChangeDutyCycle(255)
    a2.ChangeDutyCycle(255)

def right_stop():#E=0 P=0?の時
    a1.ChangeDutyCycle(0)
    a2.ChangeDutyCycle(0)


x = int(input())

if x > 0:
    right_forward()

elif x == 0:
    right_stop()

elif x < 0:
    right_back()







