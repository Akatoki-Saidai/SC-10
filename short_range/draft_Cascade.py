#もし精度の高いカスケード分類器を作れた時用
#cascade_LBP(正解画像30枚を3万枚,不正解画像2万枚)は明らかに過学習になってしまった
import cv2
import numpy as np
import RPi.GPIO as GPIO #GPIO用
import time
import sys

#(動作を止める際にはMaskの映像が動いていて、なおかつMaskのウィンドウをクリックした状態で"q"を押さないと止まらない)
#現在は顔の認識で代用

#用いるデータ(カスケード分類器)のパス
cascade = cv2.CascadeClassifier("C:\\Users\\konan\\Desktop\\haarcascade_frontalface_alt.xml")

#カメラの映像取得
cap = cv2.VideoCapture(0)

#保存する動画のfps
fps = 30

# 録画する動画のフレームサイズ
size = (640, 480)

# 出力する動画ファイルの設定（映像を取得して処理するスピードとfpsがかみ合わないと、保存した映像が早送りになってしまう。安いラズパイ用のカメラだと12fpsくらいじゃないと動画が早送りになる。)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video = cv2.VideoWriter('output.avi', fourcc, fps, size)

short_range_checked = False #近距離フェーズに入ったか否か(これを用いて近距離フェーズで一回のみ実行するコードを書く)


#-------------モーター関係----------------------------------------------------------
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



#PIN指定
AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33

#GPIOのモード
GPIO.setmode(GPIO.BOARD)#物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)#←ここでエラーoutがだめ？→out×,,OUT〇
GPIO.setup(AIN2,GPIO.OUT)
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

duty = 100 #duty比　回転速度変更用変数
#DDRV8355 = MODE0 
#--------------右モータ関数--------------
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
def forward():#前進
    right_forward()
    left_forward()



def CW():#右回転
    right_back()
    left_forward()



def CCW():#左回転
    right_forward()
    left_back()



def stop():#停止
    right_stop()
    left_stop()
    


def back():#後進
  right_back()
  left_back()

#--------------------------------------------------------------------------------



#---------------赤色検出関係-----------------------------------------------------

#赤色検知を行う関数
def red_detect(img):
    # HSV色空間に変換(ここでは、取得した映像を処理するために、RGBからHSVに変更するのと、赤色と認識するHSVの値域を設定してます。RGBからHSVにするのは、人間が見る色の感覚とと近いから)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1(OpenCVのHSVの範囲は普通の範囲と違うので、ここではOpenCVで赤とされる0～30 150～179が色相としている）[色相,彩度,明度]
    hsv_min = np.array([0, 127, 0])
    hsv_max = np.array([30, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)#inRangeは映像を2値化する関数(二値化するのはカラーより情報量が3分の1だから) [多次元配列(画像情報),２値化する条件の下限,２値化する条件の上限]

    # 赤色のHSVの値域2
    hsv_min = np.array([150, 127, 0])
    hsv_max = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

    return mask1 + mask2



#　ブロブ解析(二値化処理後の画像の中に表示されるものの形状を解析)
def analysis_blob(binary_img):
    #違う関数で使いたいから、グローバルに。
    global label
    global data
    global max_index
    global center
    # 2値画像のラベリング処理(ラベリング処理は、認識した物体の輪郭を番号づけする処理。することで物体の形を認識できる)
    #connectedComponentsWithStatsはオブジェクトのサイズや重心などの情報も合わせて返してくれる関数
    label = cv2.connectedComponentsWithStats(binary_img)

    # ブロブ情報を項目別に抽出
    n = label[0] - 1
    data = np.delete(label[2], 0, 0)
    center = np.delete(label[3], 0, 0)

    # 配列の次元数を取得
    dimensions = data.shape

    # dimensionsが空ではない
    if dimensions:
        # 2次元以上であること。※data[:,4]より2次元目のindex=4を参照しているため
        if len(dimensions) >= 2:
            # 2次元目の要素数を確認
            dim2nd = dimensions[1]
            # 2次元目の要素数5以上ならdata[:,4]の2次元目のindex=4の条件を満たす
            if dim2nd >= 5:
                # ブロブ面積最大のインデックス
                max_index = np.argmax(data[:, 4])
                # 面積最大ブロブの情報格納用
                maxblob = {}

                # 面積最大ブロブの各種情報を取得
                maxblob["upper_left"] = (data[:, 0][max_index],
                                         data[:, 1][max_index])  # 左上座標
                maxblob["width"] = data[:, 2][max_index]  # 幅
                maxblob["height"] = data[:, 3][max_index]  # 高さ
                maxblob["area"] = data[:, 4][max_index]   # 面積   面積は認識した物体ののピクセル数 / 全体のピクセル数で割り出される
                maxblob["center"] = center[max_index]  # 中心座標

                return maxblob #取得したブロブの情報を返す

    
    raise ValueError

#--------------------------------------------------------------------------



#--------------------メインとなる部分--------------------------------------

#赤色認識を行う関数とブロブ解析を行う関数を実行し、結果表示する関数
def Color_processing():
    
    while(cap.isOpened()):
        # フレームを取得
        ret, frame = cap.read()#capに入れられた、カメラの映像のデータを入れる。 retは映像を取得できたかどうかをboolでいれられる

        # 赤色検出(取得できた映像を赤色検出の関数にまわす)
        mask = red_detect(frame)
        
        try:


            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            target = analysis_blob(mask)
            #最初にカメラが起動した際、物体の面積が40000未満ならrecognition関数を実行
            if (data[:, 4][max_index] < 40000): 
                Image_processing()
            else:
                pass
            # 面積最大ブロブの中心座標を取得
            center_x = int(target["center"][0])
            center_y = int(target["center"][1])

            # フレームに面積最大ブロブの中心周囲を円で描く
            cv2.circle(frame, (center_x, center_y), 50, (0, 200, 0),
                       thickness=3, lineType=cv2.LINE_AA)

            #面積と座標表示
            print("menseki",data[:, 4][max_index])
            print(center[max_index][0])
         
            # 結果表示
            cv2.imshow("Frame", frame)
            cv2.imshow("Mask", mask)

             # 書き込み
            video.write(frame)
        
        except ValueError:
            continue


        #モーター制御挿入
        #カメラの映像は横640にしてるので、320付近がカメラの中心

#---------------------------基本的に変更を加えるところはここ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        if  270 <= center[max_index][0] and center[max_index][0] < 370 and 50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
            #まっすぐ進み動作(物体が中心近くにいる際)
            forward()
        
        elif data[:, 4][max_index] <= 50:
            #回転動作("赤い物体を検出できなくなった際")
            CW()
        
        elif center[max_index][0] < 270 and 50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
            #回転する動作(物体がカメラの中心から左にずれている際)
            CCW()
        
        elif center[max_index][0] >= 370 and 50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
            #回転する動作（物体がカメラの中心から右にずれている際）
            CW()

        elif data[:, 4][max_index] > 80000:
            #止まる(物体の近くに接近した際)
            stop()      

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        # qキーが押されたら途中終了
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
            
    video.release()
    cap.release()
    cv2.destroyAllWindows()



#学習したデータで物体を区別するプログラム
def Image_processing():
    while True:

        ret,frame = cap.read()
        facerect = cascade.detectMultiScale(frame)
        mask = red_detect(frame)
        video.write(frame)
        try:


            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            #recognition関数が実行された際に、そこでmain()の処理が止まり、面積が取得できなくなるので、recognitionにも赤色検知とブロブ解析を行う文を書くことで、面積を取得している
            target = analysis_blob(mask)
        except ValueError:
            continue


        for rect in facerect:
            cv2.rectangle(frame, tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]), (255,255,255), thickness=2) # 引数　rectangle(画像データ、検出した物を囲う四角の左頂点の座標、検出した物を囲う四角の右下頂点の座標、四角の色、四角の厚さ)
    
#---------------------------基本的に変更を加えるところはここ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            
            if len(facerect) > 0: #facerectに入った要素数が0以上の時(物体を認識できたとき)
                print("x座標" + str((rect[0]+rect[2])/2)) #検出した物体の中心のx座標
                #print("y座標" + str((rect[1]+rect[3])/2)) #検出した物体の中心のy座標

                
                #モーター制御挿入
                #カメラの映像は横640にしてるから、320付近がカメラの中心
                if  190 <= ((rect[0]+rect[2])/2) < 250 :
                    #まっすぐ進み動作(物体が中心近くにいる際)
                    forward()
        
        
                elif ((rect[0]+rect[2])/2) < 190 :
                    #回転する動作(物体がカメラの中心から左にずれている際)
                    CCW()
        
                elif ((rect[0]+rect[2])/2) >= 250 :
                    #回転する動作（物体がカメラの中心から右にずれている際）
                    CW()
                #実際やってみて、遠い距離でも認識できなくなり、コーンと違う方向へ行くなら、breakのところのlen(facerect) == 0を消して、こっちのほうで回転する条件として使う

#--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   
                """
                課題１
                学習したデータの準備・・・正解画像7000枚 不正解3000枚 用意する もしくは100枚の画像を7000枚でもかさ増しする
                課題２
                全体が写らないと認識しない・・・学習データしだいではできると思うが、学習したデータを作る難易度がかなり高い(学習した結果が予想がつかないため)
                課題３
                データを学習されるためのハードウェアの準備・・・1枚の画像を1000枚にかさ増しさせるのにi7-1195G7で二日間したが、全然終わらなそうだったのであきらめた。ハイスペックなデスクトップでは自分のノートの1/10とかで終わらせれるかもしれないが、結局何日で終わるかはわからない。
                課題4
                角度によっては認識できない・・・学習させる画像に、CanSatで実際に撮った写真を使うのがいいとは思うが現状厳しい
                課題5
                個人的な問題・・・完璧にdetectMultiScaleの引数とかを把握できてない
            
                色の認識は精度の改善が色の範囲を絞るくらいしかないが、ハードルが低い
                """
        cv2.imshow("Image_processing",frame)


        #認識できなくなるか、面積が50000を超えると停止して、色の認識に変更(40000未満になれば、また実行される)
        #なぜかcv2.waitKey(25) & 0xFF == ord('w')を消すとうまく動作しなくなる
        if (cv2.waitKey(25) & 0xFF == ord('w') or  data[:, 4][max_index] >50000  or len(facerect) == 0):
            break

#--------------------------------------------------------------------------



while True:
    #近距離フェーズ
    if reading_ultrasound_distance(0) < 450 and reading_ultrasound_distance(0) != 0: #または画像認識の可能な距離に入る(gps_distance < x)

        """
        実機が完成してきて、実際に走る際に取得できる面積や座標の値がわかってきたら、モーターの制御を加えたいと思ってます。
        実際に走る際にへこみとかで、機体の動作がずれて、カメラが物体を認識できなくなっても、１～４０くらいの範囲で面積が取得されるので、それを条件として、回転して物体を再発見するようにしたいです。
        会場がどんな感じかはわからないですが、赤い服の人が移ったり、自然にある赤色に反応してしまわないかが懸念点です。
        """
        

        #近距離フェーズに入ってから一回のみ実行する
        if not short_range_checked:

            #一回のみ実行する操作をここに書く
            Color_processing()

            short_range_checked = True


        #近距離フェーズで繰り返し行われる操作をここに書く

        if #ゴール条件:
            #ゴール判定
