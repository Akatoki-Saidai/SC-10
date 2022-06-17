import time
import RPi.GPIO as GPIO


short_range_checked = False #近距離フェーズに入ったか否か(これを用いて近距離フェーズで一回のみ実行するコードを書く)


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


while True:
    #近距離フェーズ
    if reading_ultrasound_distance(0) < 450 and reading_ultrasound_distance(0) != 0: #または画像認識の可能な距離に入る(gps_distance < x)
        

        #近距離フェーズに入ってから一回のみ実行する
        if not short_range_checked:

            #一回のみ実行する操作をここに書く

            short_range_checked = True


        #近距離フェーズで繰り返し行われる操作をここに書く

        if #ゴール条件:
            #ゴール判定



