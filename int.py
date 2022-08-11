from lzma import PRESET_DEFAULT
from micropyGPS import MicropyGPS
import pigpio
from math import *
import cv2
import numpy as np
import RPi.GPIO as GPIO #GPIOインポート
import time #時間制御インポート
import threading
import csv
import atexit
import smbus
import datetime
import sys
from Adafruit_BNO055 import BNO055
import logging
import subprocess


#bnosetup
bno = BNO055.BNO055(rst=18)
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':# パラメータとして-vが渡された場合、冗長なデバッグロギングを有効にする
    logging.basicConfig(level=logging.DEBUG)
if not bno.begin():# BNO055を初期化し、問題が発生した場合は停止する
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
status, self_test, error = bno.get_system_status()# システムの状態やセルフテストの結果を表示する
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
if status == 0x01:# システムステータスがエラーモードの場合、エラーを表示する
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')
sw, bl, accel, mag, gyro = bno.get_revision()# Print BNO055 software revision and other diagnostic data.
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

##bme280setup
bus_number  = 1
i2c_address = 0x76
bus = smbus.SMBus(bus_number)
digT = []
digP = []
digH = []
t_fine = 0.0

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


def writeReg(reg_address, data):
	bus.write_byte_data(i2c_address,reg_address,data)

def get_calib_param():
	calib = []
	for i in range (0x88,0x88+24):
		calib.append(bus.read_byte_data(i2c_address,i))
	calib.append(bus.read_byte_data(i2c_address,0xA1))
	for i in range (0xE1,0xE1+7):
		calib.append(bus.read_byte_data(i2c_address,i))
	digT.append((calib[1] << 8) | calib[0])
	digT.append((calib[3] << 8) | calib[2])
	digT.append((calib[5] << 8) | calib[4])
	digP.append((calib[7] << 8) | calib[6])
	digP.append((calib[9] << 8) | calib[8])
	digP.append((calib[11]<< 8) | calib[10])
	digP.append((calib[13]<< 8) | calib[12])
	digP.append((calib[15]<< 8) | calib[14])
	digP.append((calib[17]<< 8) | calib[16])
	digP.append((calib[19]<< 8) | calib[18])
	digP.append((calib[21]<< 8) | calib[20])
	digP.append((calib[23]<< 8) | calib[22])
	digH.append( calib[24] )
	digH.append((calib[26]<< 8) | calib[25])
	digH.append( calib[27] )
	digH.append((calib[28]<< 4) | (0x0F & calib[29]))
	digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
	digH.append( calib[31] )
	for i in range(1,2):
		if digT[i] & 0x8000:
			digT[i] = (-digT[i] ^ 0xFFFF) + 1
	for i in range(1,8):
		if digP[i] & 0x8000:
			digP[i] = (-digP[i] ^ 0xFFFF) + 1
	for i in range(0,6):
		if digH[i] & 0x8000:
			digH[i] = (-digH[i] ^ 0xFFFF) + 1 
      
def compensate_P(adc_P):
	global  t_fine
	pressure = 0.0
	v1 = (t_fine / 2.0) - 64000.0
	v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
	v2 = v2 + ((v1 * digP[4]) * 2.0)
	v2 = (v2 / 4.0) + (digP[3] * 65536.0)
	v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144
	v1 = ((32768 + v1) * digP[0]) / 32768
	if v1 == 0:
		return 0
	pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
	if pressure < 0x80000000:
		pressure = (pressure * 2.0) / v1
	else:
		pressure = (pressure / v1) * 2
	v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
	v2 = ((pressure / 4.0) * digP[7]) / 8192.0
	pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)  
	return pressure/100
  
def compensate_T(adc_T):
	global t_fine
	v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
	v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
	t_fine = v1 + v2
	temperature = t_fine / 5120.0
	return temperature

def compensate_H(adc_H):
	global t_fine
	var_h = t_fine - 76800.0
	if var_h != 0:
		var_h = (adc_H - (digH[3] * 64.0 + digH[4]/16384.0 * var_h)) * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)))
	else:
		return 0
	var_h = var_h * (1.0 - digH[0] * var_h / 524288.0)
	if var_h > 100.0:
		var_h = 100.0
	elif var_h < 0.0:
		var_h = 0.0
	return var_h


#PIN指定
AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33

#GPIOのモード

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


def setup():
	osrs_t = 1			#Temperature oversampling x 1
	osrs_p = 1			#Pressure oversampling x 1
	osrs_h = 1			#Humidity oversampling x 1
	mode   = 3			#Normal mode
	t_sb   = 5			#Tstandby 1000ms
	filter = 0			#Filter off
	spi3w_en = 0			#3-wire SPI Disable
	ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
	config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
	ctrl_hum_reg  = osrs_h
	writeReg(0xF2,ctrl_hum_reg)
	writeReg(0xF4,ctrl_meas_reg)
	writeReg(0xF5,config_reg)

setup()
get_calib_param()

##GPSsetup
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
my_gps = MicropyGPS(9, 'dd')#my_gpsにデータが格納される感じ

tm_last = 0 # 10秒ごとに表示 #なんか10秒ごとに表示されてない気がするのでいじってみてほしい
count = 0

#servo setup
Servo_pin = 18                      #変数"Servo_pin"に18を格納
#GPIO.setmode(GPIO.BCM)              #GPIOのモードを"GPIO.BCM"に設定
GPIO.setup(Servo_pin, GPIO.OUT)     #GPIO18を出力モードに設定
Servo = GPIO.PWM(Servo_pin, 50)     #GPIO.PWM(ポート番号, 周波数[Hz])

def servo_angle(angle):#角度からデューティ比を求める関数
    duty = 2.5 + (12.0 - 2.5) * (angle + 90) / 180   #角度からデューティ比を求める
    Servo.ChangeDutyCycle(duty)     #デューティ比を変更
    time.sleep(0.3)                 #0.3秒間待つ





#equation
def caliculate_h():
    p0 = 1013.25
    p = compensate_P(pres_raw)
    t = compensate_T(temp_raw)
    h = ((p0/p)**(1/5.257)-1)*(t + 273.15) / 0.0065
    return h

goal_latitude = 0 #実測する
goal_longitude  =  0 #実測する
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
    d = r*acos(sin(ap)*sin(bp)+cos(bp)*cos(b)*cos(a-ap))
    return d



##integrate all sections##

with open('bno_data1.csv', 'w') as fbno1, open('bno_data2.csv','w') as fbno2, open('bno_data3.csv','w') as fbno3, open('bno_data4.csv','w') as fbno4, open('bno_data5.csv','w') as fbno5, open('bme_data.csv','w') as fbme, open('gps_data.csv','w') as fg, open('log_data.csv','w') as fl:
    #csv書き込み開始
    bno0551 = csv.writer(fbno1)
    bno0551.writerow(['now','magx','magy','magz'])
    bno0552 = csv.writer(fbno2)
    bno0552.writerow(['now','Gyx','Gyy','Gyz'])
    bno0553 = csv.writer(fbno3)
    bno0553.writerow(['now','ax','ay','az'])
    bno0554 = csv.writer(fbno4)
    bno0554.writerow(['now','heading','roll','pitch'])
    bno0555 = csv.writer(fbno5)
    bno0555.writerow(['now','lx','ly','lz'])
    bme280 = csv.writer(fbme)
    bme280.writerow(['now','temp','pres','hum'])
    gps = csv.writer(fg)
    gps.writerow(['now','latitude','longitude','altitude'])
    log_data = csv.writer(fl)
    phase = 1
    while True:
        heading, roll, pitch = bno.read_euler()# Read the Euler angles for heading, roll, pitch (all in degrees).
        sys, gyro, accel, mag = bno.get_calibration_status() # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        qx,qy,qz,qw = bno.read_quaternion()        # Orientation as a quaternion:
        temp_c = bno.read_temp()        # Sensor temperature in degrees Celsius:
        mx,my,mz = bno.read_magnetometer()        # Magnetometer data (in micro-Teslas):
        Gx,Gy,Gz = bno.read_gyroscope()        # Gyroscope data (in degrees per second):
        ax,ay,az = bno.read_accelerometer()        # Accelerometer data (in meters per second squared):
        lx,ly,lz = bno.read_linear_acceleration()        # Linear acceleration data (i.e. acceleration from movement, not gravity--returned in meters per second squared):
        gx,gy,gz = bno.read_gravity()        # Gravity acceleration data (i.e. acceleration just from gravity--returned in meters per second squared):
        get_calib_param()
        data = []
        for i in range (0xF7, 0xF7+8):
            data.append(bus.read_byte_data(i2c_address,i))
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw  = (data[6] << 8)  |  data[7]
        (count, sentence) = serialpi.bb_serial_read(RX)#ここはよくわからん、データが取れてるかどうか調べるところな気がする
        if len(sentence) > 0:
            for x in sentence:
                if 10 <= x <= 126:
                    stat = my_gps.update(chr(x))
                    if stat:
                        tm = my_gps.timestamp
                        gps.writerow([my_gps.latitude[0], my_gps.longitude[0], my_gps.altitude])
        get_calib_param()

        now = datetime.datetime.now
        time_sta = time.perf_counter()
        bnodata1 = [now,mx,my,mz]
        bnodata2 = [now,Gx,Gy,Gz]
        bnodata3 = [now,ax,ay,az]
        bnodata4 = [now,heading, roll, pitch]
        bnodata5 = [now,lx,ly,lz]
        t_p_h = [compensate_T(temp_raw), compensate_P(pres_raw), compensate_H(hum_raw)]
        bno0551.writerow(bnodata1)
        bno0552.writerow(bnodata2)
        bno0553.writerow(bnodata3)
        bno0554.writerow(bnodata4)
        bno0555.writerow(bnodata5)
        bme280.writerow(t_p_h)
        
        goal_heading = 0
        if phase == 1: #待機フェーズ
            if ay**2 < 150: #条件は後で変更
                time.sleep(0.01)
                log_data.writerow([now,'im waiting'])
                print(ay**2,'im waiting' )
            elif  ay**2 > 150:
                time.sleep(2)
                log_data.writerow([now,'im falling'])
                print(ay**2,'im falling')
                phase += 1
                
            #elif :#計測開始から450秒（7分30秒）経ったら走行フェーズ開始
                 #phase += 1
        if phase == 2:#落下フェーズ
            servo_angle(-40)
            Servo.stop()
            CCW()
            if goal_heading - heading < 5:
                stop()
                phase += 1
	
	if phase == 3:
	    subprocess.run(['python3 /home/issa/e2e_improved.py'])          
            
            
