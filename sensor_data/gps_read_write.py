import time
import pigpio
from micropyGPS import MicropyGPS
import csv


def main():
    # シリアル通信設定
    baudrate = 9600
    #通信設定でいじるとしたらここのTX,RXだけど、ピン配置変えたい場合以外はいじらなくてok
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
    #my_gpsにデータが格納される感じ
    my_gps = MicropyGPS(9, 'dd')

    # 10秒ごとに表示
    #なんか10秒ごとに表示されてない気がするのでいじってみてほしい
    tm_last = 0
    count = 0
    
    with oprn('gps_data.csv', 'w') as f:#csvファイルへの書き込み(一行目)
        writer = csv.writer(f)
        writer.writerow(['latitude', 'longtitude', 'altitude'])
    while True:
        (count, sentence) = serialpi.bb_serial_read(RX)#ここはよくわからん、データが取れてるかどうか調べるところな気がする
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
                            print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0], "altitude:", my_gps.altitude)
                            with open('gps_data.csv','w') as f:#csvファイルへの書き込み(data)
                                writer = csv.writer(f)
                                writer.writerow([my_gps.latitude[0], my_gps.longitude[0], my_gps.altitude])
                            time.sleep(1)#一秒停止(試しに入れてみただけ)
"""
tmは時間に関するやつだと思うけどよく知らない。使わなくてもいい可能性がある。
my_gps.latitude[0]は緯度
my_gps.longitude[0]は経度
my_gps.altitudeは高度(未確認)
whileの中のif文はいじくりまわしても多分大丈夫だと思う。
おかしくなったら元に戻す感じで試行錯誤してみてほしいです。
"""
if __name__ == "__main__":
    main()
"""
参考
https://www.rs-online.com/designspark/raspberry-pi-2nd-uart-a-k-a-bit-banging-a-k-a-software-serial
https://raspberrypi.stackexchange.com/questions/62578/software-serial-send-with-pigpio-sending-garbage
https://abyz.me.uk/rpi/pigpio/
https://abyz.me.uk/rpi/pigpio/download.html
https://zenn.dev/kotaproj/books/raspberrypi-tips/viewer/370_kiso_gps
"""
