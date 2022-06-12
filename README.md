# SC-10
## リンク集
- [SC-10 全体議事録](https://docs.google.com/document/d/1omJv3-G53QQhIMKfowVk7e65B6BSIAqXMu1nXZjD6hc/edit)
- [構造班議事録](https://docs.google.com/document/d/1dUNLgnagP7UcUEBuSQTaFuIYSMlyMas03L2opLKp5go/edit)
- [電装班議事録](https://docs.google.com/document/d/1Y7DEvbpZ9DoRgqgOD5M4f_ljMeG6zpjPnzOObKJcrcc/edit?pli=1)
- [会計シート](https://docs.google.com/spreadsheets/d/1c_H02MdM6czNDfDs1xQuezBuoZI_cPhWze7ftNqSkN4/edit#gid=0)

## 仕様
<table>
  <tr>
    <td width="20%">種類</td>
    <td width="40%">商品名</td>
    <td width="40%">購入元リンク</td>
  <tr>
    <td>9軸センサ</td>
    <td>BNO055</td>
    <td>https://akizukidenshi.com/catalog/g/gK-16996/</td>
  </tr>
  <tr>
    <td>GPS</td>
    <td>ＧＰＳ受信機キット　１ＰＰＳ出力付き　<br>「みちびき」２機受信対応</td>
    <td>https://akizukidenshi.com/catalog/g/gK-09991/</td>
  </tr>
  <tr>
    <td>気圧．温度センサ</td>
    <td>BME280</td>
    <td>https://www.switch-science.com/catalog/2236/</td>
  </tr>
  <tr>
    <td>超音波センサ</td>
    <td>HC-SR04</td>
    <td>https://akizukidenshi.com/catalog/g/gM-11009/</td>
  </tr>
  <tr>
    <td>通信</td>
    <td>TWE-Lite dip</td>
    <td>https://akizukidenshi.com/catalog/g/gM-06760/</td>
  </tr>
  <tr>
    <td>空気質センサ</td>
    <td>CCS811</td>
    <td>https://www.switch-science.com/catalog/3298/</td>
  </tr>
  <tr>
    <td>サーボモーター</td>
    <td>SF180M</td>
    <td>https://www.amazon.co.jp/dp/B0852Z9ZR4?ref_=cm_sw_r_cp_ud_dp_796KC9HYB95R2NAKP0CZ</td>
  </tr>
</table>

## Raspberry Pi セットアップ方法
coming soon...（担当：土橋 or 宮崎）

## ソフトシリアル使い方(暫定
#### pigpioのインストール
[pigpio download](http://abyz.me.uk/rpi/pigpio/download.html)よりpigpioをインストールします
Dwnload and install latest versionに従い以下のコマンドをターミナルで順に実行します。($マーク以降を入力)
makeコマンドはそこそこ時間がかかります。
```
~$ wget https://github.com/joan2937/pigpio/archive/master.zip  
~$ unzip master.zip  
~$ cd pigpio-master  
~/pigpio-master$ make  
~/pigpio-master$ sudo make install
```
完了したら、pigpio.pyをプログラムが実行されるディレクトリにコピーします。
以下をターミナルで実行。
(GUI環境で操作する場合は単にファイルをコピーペーストでok)
```
~/pigpio-master$ cp pigpio.py コピー先のディレクトリのアドレス
```
#### pigpio daemonの起動
pigpioを使用する際は**pigpio daemonを起動させる必要**があります。
これは**raspiを起動するごとに終了してしまう**ので、常に起動させておきたい場合はraspiを起動する際にpigpio daemonを起動させるように**設定する必要**があります。
pigpio daemonを起動させるコマンドは以下の通りです。
```
sudo pigpiod
```
#### ソースコードの記述
以下はサンプルコードです。
```python
import pigpio



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

count = 0

(count, sentence) = serial.bb_serial_read(RX)
```
baudrateはこれ以上の値にすると**うまくデータが受け取れない**可能性があります。
**TXとRXにはGPIO番号を代入**することでそのpinを用いてシリアル通信を行うことが出来ます。
サンプルコードのTX、RXは単なる変数であるので、**変数名の変更は可能**です。
`(count, sentence) = serial.bb_serial_read(RX)`は`sentence`が0より大きいかどうかを調べることでデータが受け取れているかを調べることが出来ます。
データが受け取れていれば`sentence > 0`は`True`です。
また、未検証ですが**複数のsoftware serialを用いて通信することも可能**だと思います。（担当：高野真）

## 画像認識まとめ
coming soon...（佐々木君お願いできますか？）

## ピン配置
- Raspberry Pi 3B
<img src="https://www.bigmessowires.com/wp-content/uploads/2018/05/Raspberry-GPIO.jpg" alt="Raspberry Pi" title="Raspberry Pi ピン配置">

- TWE-Lite
<img src="https://cdn-ak.f.st-hatena.com/images/fotolife/n/nobita_RX7/20190803/20190803032714.jpg" alt="TWE-Lite" title="TWE-Lite ピン配置">
