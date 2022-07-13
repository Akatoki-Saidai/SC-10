import logging
import sys
import time

from Adafruit_BNO055 import BNO055

# csvファイル保存
import csv


 # 日時に関して
import datetime

bno = BNO055.BNO055(rst=18)

# パラメータとして-vが渡された場合、冗長なデバッグロギングを有効にする
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

    
# BNO055を初期化し、問題が発生した場合は停止する
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

    
# システムの状態やセルフテストの結果を表示する
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))


# システムステータスがエラーモードの場合、エラーを表示する
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

    
# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')
with open('log.csv', 'w') as f:   # log.csvのファイルに保存される
    while True:
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        heading, roll, pitch = bno.read_euler()
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        sys, gyro, accel, mag = bno.get_calibration_status()
        # Print everything out.
        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
            heading, roll, pitch, sys, gyro, accel, mag))
        # Other values you can optionally read:
        # Orientation as a quaternion:
        #x,y,z,w = bno.read_quaterion()
        # Sensor temperature in degrees Celsius:
        #temp_c = bno.read_temp()
        # Magnetometer data (in micro-Teslas):
        mx,my,mz = bno.read_magnetometer()
        # Gyroscope data (in degrees per second):
        Gx,Gy,Gz = bno.read_gyroscope()
        # Accelerometer data (in meters per second squared):
        ax,ay,az = bno.read_accelerometer()
        # Linear acceleration data (i.e. acceleration from movement, not gravity--
        # returned in meters per second squared):
        # x,y,z = bno.read_linear_acceleration()
        # Gravity acceleration data (i.e. acceleration just from gravity--returned
        # in meters per second squared):
        #x,y,z = bno.read_gravity()
        # Sleep for a second until the next reading.
        print ("magx=",mx,"magy=",my,"magz=",mz,"Gyx=",Gx,"Gyy=",Gy,"Gyz=",Gz,"accelx=",ax,"accely=",ay,"accelz=",az) 
        time.sleep(1)
        now = datetime.datetime.now()
        recordtime = 'rec_{0:%Y%m%d}'.format(now)
        print("recordtime = %s" % recordtime)

        data = "magx="+str(mx)+"magy="+str(my)+"magz="+str(mz)+"Gyx="+str(Gx)+"Gyy="+str(Gy)+"Gyz="+str(Gz)+"accelx="+str(ax)+"accely="+str(ay)+"accelz="+str(az)
        data = recordtime + "," + data
        data = data.split(',') 
        print("data= %s" % data)

        writer = csv.writer(f)
        writer.writerow(data)

        f.close()
        print("loggging done. see log.csv")
#参考https://qiita.com/kmaepu/items/779ab8e45bfe96230224
