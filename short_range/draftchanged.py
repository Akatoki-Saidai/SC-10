import time
import RPi.GPIO as GPIO
import cv2
import numpy as np


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


def red_detect(img):
    # HSV色空間に変換(ここでは、取得した映像を処理するために、RGBからHSVに変更するのと、赤色と認識するHSVの値域を設定してます。)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1(OpenCVのHSVの範囲は普通の範囲と違うので、ここではOpenCVで赤とされる0～30 150～179が色相としてます）[色相,彩度,明度]
    hsv_min = np.array([0, 127, 0])
    hsv_max = np.array([30, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max) #inRangeは映像を2値化する関数です(二値化するのはカラーより情報量が3分の1だからです) [多次元配列(画像情報),２値化する条件の下限,２値化する条件の上限]

    # 赤色のHSVの値域2
    hsv_min = np.array([150, 127, 0])
    hsv_max = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

    return mask1 + mask2

# ブロブ解析(二値化処理後の画像の中に表示されるものの形状を解析)

def analysis_blob(binary_img):
    #違う関数で、変数を使いたいので、グローバルにしてます。
    global label
    global data
    global max_index
    global center
    # 2値画像のラベリング処理(ラベリング処理は、認識した物体の輪郭を番号づけする処理です。することで物体の形を認識できます)
    #connectedComponentsWithStatsはオブジェクトのサイズや重心などの情報も合わせて返してくれる関数です。
    label = cv2.connectedComponentsWithStats(binary_img)

    # ブロブ情報を項目別に抽出
    n = label[0] - 1
    data = np.delete(label[2], 0, 0)
    center = np.delete(label[3], 0, 0)

    # 配列の次元数を取得
    dimensions = data.shape

    # dimensionsが空ではない
    if dimensions:
        # 2次元以上であること。※data[:,4]（物体の面積が入れらえれている4番目の列）より2次元目のindex=4を参照しているため。2次元以上じゃないと、面積が取得できません
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
                                         data[:, 1][max_index])  # 左上を(0,0)とした座標
                maxblob["width"] = data[:, 2][max_index]  # 幅
                maxblob["height"] = data[:, 3][max_index]  # 高さ
                maxblob["area"] = data[:, 4][max_index]   # 面積　面積は認識した物体ののピクセル数 / 全体のピクセル数で出ています
                maxblob["center"] = center[max_index]  # 中心座標

                return maxblob #取得したブロブの情報を返してあげます

def Image_processing():
   
    # カメラのキャプチャ
    cap = cv2.VideoCapture(0)

    fps = 30

    # 録画する動画のフレームサイズ（webカメラと同じにする）
    size = (640, 480)

    # 出力する動画ファイルの設定（映像を取得して処理するスピードとfpsがかみ合わないと、保存した映像が早送りになってしまうのですが、安いラズパイ用のカメラでは12fpsくらいじゃないと早送りになってしまいました)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video = cv2.VideoWriter('CameraLog.avi', fourcc, fps, size)

    while(cap.isOpened()):
        # フレームを取得
        ret, frame = cap.read() #capに入れられた、カメラの映像を入れます

        # 赤色検出
        mask = red_detect(frame) #赤色検出の関数に読み込んだカメラの映像のデータを入れて処理します

        #物体が検出されないと、次元数が2未満になり、ValueErrorが起きるので、起きたらフレームを飛ばしてます
        try:

            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            target = analysis_blob(mask)

            # 面積最大ブロブの中心座標を取得
            center_x = int(target["center"][0])
            center_y = int(target["center"][1])

            # フレームに面積最大ブロブの中心周囲に円を描く
            cv2.circle(frame, (center_x, center_y), 50, (0, 200, 0),
                       thickness=3, lineType=cv2.LINE_AA)

            
            print("menseki",data[:, 4][max_index])
            print(center[max_index][0])
            # window表示
            cv2.imshow("Frame", frame)
            cv2.imshow("Mask", mask)

            # 動画ファイルの書き込み
            video.write(frame)
        
        except ValueError:
            continue

　　　　    #モーター制御挿入
        #カメラの映像は横640にしてるので、320付近がカメラの中心です
        if  270 <= center[max_index][0] and center[max_index][0] < 370:
            #まっすぐ進み動作(物体が中心近くにいる際)
            print("まっすぐ")
        
        elif data[:, 4][max_index] < 50:
            #回転動作("赤い物体を検出できなくなった際")
            pritn("回転")
        
        elif center[max_index][0] < 270:
            #回転する動作(物体がカメラの中心から左にずれている際)
            print("右回転")
        
        elif center[max_index][0] >= 370:
            #回転する動作（物体がカメラの中心から右にずれている際）
            print("左回転")

        elif data[:, 4][max_index] > 80000:
            #止まる(物体の近くに接近した際)
            print("止まる")
　　　　#物体の面積の値によって、モーターのスピードを変化させれたらと思ったのですが、いい感じに変化する式がわからなかったのであきらめました。

        # qキーが押されたら途中終了(何かあった時ように、手動停止)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    cap.release()
    video.release()
    cv2.destroyAllWindows()

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
      
            Image_processing()

            short_range_checked = True


        #近距離フェーズで繰り返し行われる操作をここに書く

        if #ゴール条件:
            #ゴール判定
