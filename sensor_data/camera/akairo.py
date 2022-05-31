#-*- coding: utf-8 -*-

import cv2

import numpy as np

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

GPIO.setup(12, GPIO.OUT)

GPIO.setup(16, GPIO.OUT)

GPIO.setup(18, GPIO.OUT)

GPIO.setup(33, GPIO.OUT)

def red_detect(img):

    # HSV色空間に変換

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)



    # 赤色のHSVの値域1

    hsv_min = np.array([0,127,0])

    hsv_max = np.array([30,255,255])

    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)



    # 赤色のHSVの値域2

    hsv_min = np.array([150,127,0])

    hsv_max = np.array([179,255,255])

    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)



    return mask1 + mask2



# ブロブ解析

def analysis_blob(binary_img):

    # 2値画像のラベリング処理

    label = cv2.connectedComponentsWithStats(binary_img)



    # ブロブ情報を項目別に抽出

    n = label[0] - 1

    global data #変数をローカルスコープの外でも使いたくて、global宣言してます

    data = np.delete(label[2], 0, 0)

    center = np.delete(label[3], 0, 0)



    # ブロブ面積最大のインデックス
    global max_index#上と同様です

    max_index = np.argmax(data[:, 4])



    # 面積最大ブロブの情報格納用

    maxblob = {}



    # 面積最大ブロブの各種情報を取得

    maxblob["upper_left"] = (data[:, 0][max_index], data[:,1][max_index]) # 左上座標

    maxblob["width"] = data[:, 2][max_index]  # 幅

    maxblob["height"] = data[:, 3][max_index]  # 高さ

    maxblob["area"] = data[:, 4][max_index]  # 面積

    maxblob["center"] = center[max_index]  # 中心座標



    return maxblob



def main():

    videofile_path ="C:/github/sample/python/opencv/video/color_tracking/red_pendulum.mp4"



    # カメラのキャプチャ

    camera = cv2.VideoCapture(0)



    while(camera.isOpened()):

    # フレームを取得

        ret, frame = camera.read()



    # 赤色検出

        mask = red_detect(frame)


    # マスク画像をブロブ解析（面積最大のブロブ情報を取得）

        target = analysis_blob(mask)



    # 面積最大ブロブの中心座標を取得
        center_x = int(target["center"][0])
        center_y = int(target["center"][1])
       
     #radiusは整数値じゃないといけないみたいなので、仮として、受け取った面積の割る1000し、切り捨てした値にしてみました。
        
        radius = ((data[:,4][max_index]) // 1000)

     # フレームに面積最大ブロブの中心周囲を円で描く
        cv2.circle(frame, (center_x, center_y), radius, (0, 200, 0)
                  thickness=3, lineType=cv2.LINE_AA)



     # 結果表示

         cv2.imshow("Frame", frame)

         cv2.imshow("Mask", mask)

     #radiusの値をいろいろ試して、ちょうどいいところを見つけたいと思います。そして、モーターの制御は今は簡易的に書いています。
         if(radius < 100):

             GPIO.output(12,1)

             GPIO.output(33,1)

         if(radius > 100):

             GPIO.output(12,0)

             GPIO.output(33,0)

        # qキーが押されたら途中終了

            if cv2.waitKey(25) & 0xFF == ord('q'):

                break

        cap.release()
        cv2.destroyAllWindows()
if__name__ =='__main__':
    main()
