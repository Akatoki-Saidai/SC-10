import serial

s = serial.Serial("/dev/serial0",115200,timeout=10) #serial通信の設定、"serialポート設定,通信速度,待つ時間(初期設定は∞,0にすると全く待たない．)"
while True:
    if s.in_waitng > 0:
        resv_data = s.read(1)
        print(type(recv_data))
        a = struct.unpack_from("B",recv_data ,0)
        print(a)
        print("success!!") #finish sending

#参考
#http://www.s12600.net/psy/python/21-2.html
#https://qiita.com/umi_mori/items/757834e0ef75f38cea19
