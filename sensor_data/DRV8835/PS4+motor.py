
import RPi.GPIO as GPIO #GPIO用
import time

#PIN指定
AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33

#GPIOのモード
GPIO.setmode(GPIO.BOARD)    #物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)   #a1ばんピンでGPIO
GPIO.setup(AIN2,GPIO.OUT)   #a2ばんピンでGPIO(PWM)
GPIO.setup(BIN1,GPIO.OUT)   #b1ばんピンでGPIO
GPIO.setup(AIN2,GPIO.OUT)   #b2ばんピンでGPIO(PWM)

#周波数設定
Apwm = GPIO.PWM(AIN2,200)     #a1で100HzPWM
Bpwm = GPIO.PWM(BIN2,200)     #b1で100HzPWM

def forward():                #前進
    Apwm.start(0)             #右車輪(1,1) 
    GPIO.output(AIN1,GPIO.HIGH)
    Apwm.ChangeDutyCycle(75)
    Bpwm.start(0)             #左車輪(1,1)
    GPIO.output(BIN1,GPIO.HIGH)
    Bpwm.ChangeDutyCycle(75)
    
def turn_right():                   #右回転
    Apwm.start(0)                  #右車輪(1,1)
    GPIO.output(AIN1,GPIO.HIGH)
    Apwm.ChangeDutyCycle(75)
    Bpwm.start(0)                  #左車輪(1,0)
    GPIO.output(BIN1,GPIO.HIGH)
    Bpwm.ChangeDutyCycle(0)
    
def turn_left():                   #左回転
    Apwm.start(0)                  #右車輪(1,0)
    GPIO.output(AIN1,GPIO.HIGH)
    Apwm.ChangeDutyCycle(0)
    Bpwm.start(0)                  #左車輪(1,1)
    GPIO.output(BIN1,GPIO.HIGH)
    Bpwm.ChangeDutyCycle(75)

def back():                        #後進
    Apwm.start(0)                  #右車輪(0,1)
    GPIO.output(AIN1,GPIO.LOW)
    Apwm.ChangeDutyCycle(75)
    Bpwm.start(0)                  #左車輪(0,1)
    GPIO.output(BIN1,GPIO.LOW)
    Bpwm.ChangeDutyCycle(75)
    
def stop():                        #停止
    Apwm.start(0)                  #右車輪(0,0)
    GPIO.output(AIN1,GPIO.LOW)
    Apwm.ChangeDutyCycle(0)
    Bpwm.start(0)                  #左車輪(0,0)
    GPIO.output(BIN1,GPIO.LOW)
    Bpwm.ChangeDutyCycle(0)
    
    
    
    
def def_test():
    while True:
        act = input("動作を入力>>>")
        if act == "f":
            forward()
            time.sleep(2)
            stopping()
            break
        elif act =="l":
            turn_left()
            time.sleep(2)
            Apwm.stop()
            Bpwm.stop()
        elif act =="r":
            turn_right()
            time.sleep(2)
            Apwm.stop()
            Bpwm.stop()
        elif act =="b":
            back()
            time.sleep(2)
            Apwm.stop()
            Bpwm.stop()        
        else:
            print("終了します。")
            GPIO.cleanup()

from pyPS4Controller.controller import Controller
class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs) 
          
    def on_triangle_press(self):#前進 
        forward()              
    def on_x_press(self):   #後進
        back()
    def on_square_press(self):#右回転
        turn_right()       
    def on_circle_press(self):#左回転
        turn_left()

    def on_triangle_release(self):
        stop()
    def on_x_release(self):
        stop()
    def on_square_release(self):
        stop()
    def on_circle_release(self):
        stop()
        
    def on_home_press():
        def_test()
        
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
    

        
        
        




