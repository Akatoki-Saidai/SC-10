
#モジュールインポート
import RPi.GPIO as GPIO #GPIO用
import time
from time import sleep 
import sys
from pyPS4Controller.controller import Controller#PS４コントローラ用

#PIN指定
AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33

#GPIOのモード
GPIO.setmode(GPIO.BOARD)#物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)#←ここでエラーoutがだめ？→out×,,OUT〇
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
def forward():#前進
    right_forward()
    left_forward()

def CW():#右回転
    right_back()
    left_forward()

def CCW():#左回転
    right_forward()
    left_back()

def stop():#停止
    right_stop()
    left_stop()
    
def back():#後進
  right_back()
  left_back()
    
    
    
    
#↑は全部初期設定と関数の宣言
    
    
#--------ps4コントローラの利用操作---------


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
#モーター動かす   
    def on_triangle_press(self):#前進 なぜかdef <ボタン名称>　命令　の形以外を繰り返し処理してくれない
        forward()   　           #どこにGPS、地磁気のコード書けばいいの？
        
    def on_x_press(self):
        back()
        
    def on_square_press(self):#反時計回り
        CCW()
        
    def on_circle_press(self):#時計回り
        CW()
        
#モーター止める
    def on_triangle_release(self):
        stop()
    def on_x_release(self):
        stop()
    def on_square_release(self):
        stop()
    def on_circle_release(self):
        stop()
        
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()
''''
どうゆう手順で処理されてるかわかんないけどこれでモータは動いた
どうにかGPSと地磁気も組み込みたい
''''
    
