import time
import pigpio
from micropyGPS import MicropyGPS

def main():    
    # シリアル通信設定
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
    # gps設定
    my_gps = MicropyGPS(9, 'dd')

    # 10秒ごとに表示
    tm_last = 0
    count = 0
    while True:
        (count, sentence) = serialpi.bb_serial_read(RX)
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
                            print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0])

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
