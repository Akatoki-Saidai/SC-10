import logging
import sys
import time
import numpy as np
from Adafruit_BNO055 import BNO055

# csvファイル保存
import csv


 # 日時に関して
import datetime

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
while True:
    heading, roll, pitch = bno.read_euler()# Read the Euler angles for heading, roll, pitch (all in degrees).
    sys, gyro, accel, mag = bno.get_calibration_status() # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    qx,qy,qz,qw = bno.read_quaterion()        # Orientation as a quaternion:
    temp_c = bno.read_temp()        # Sensor temperature in degrees Celsius:
    mx,my,mz = bno.read_magnetometer()        # Magnetometer data (in micro-Teslas):
    Gx,Gy,Gz = bno.read_gyroscope()        # Gyroscope data (in degrees per second):
    ax,ay,az = bno.read_accelerometer()        # Accelerometer data (in meters per second squared):
    lx,ly,lz = bno.read_linear_acceleration()        # Linear acceleration data (i.e. acceleration from movement, not gravity--returned in meters per second squared):
    gx,gy,gz = bno.read_gravity()        # Gravity acceleration data (i.e. acceleration just from gravity--returned in meters per second squared):

    x = qx / 16384
    y = qy / 16384
    z = qz / 16384
    w = qw / 16384

    ysqr = y ** 2

    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = np.arctan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    if t2 > 1.0:
        t2 = 1.0
    if t2 < -1.0:
        t2 = -1.0
    
    pitch = np.arcsin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z);  
    yaw = np.arctan2(t3, t4)

    print("roll :%f", roll)
    print("pitch:%f", pitch)
    print("yaw  :%f", yaw)
