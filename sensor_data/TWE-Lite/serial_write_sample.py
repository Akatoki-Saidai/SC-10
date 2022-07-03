import serial

s = serial.Serial("/dev/serial0",115200,timeout=10) #serial通信の設定、"serialポート設定,通信速度,待つ時間(初期設定は∞,0にすると全く待たない．)"
s.write(b'hello world')#バイナリ通信のとき，"b"を入れる
s.close()#最後にcloseすればいい

#参考
#http://www.s12600.net/psy/python/21-2.html
#https://qiita.com/umi_mori/items/757834e0ef75f38cea19
