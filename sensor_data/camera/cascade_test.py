import cv2
import numpy as np
cascade = cv2.CascadeClassifier("C:\\Users\\konan\\Desktop\\haarcascade_frontalface_alt.xml")#用いるデータのパス
cap = cv2.VideoCapture(0)
while True:
    ret,frame = cap.read()
    facerect = cascade.detectMultiScale(frame)
    
    for rect in facerect:
        cv2.rectangle(frame, tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]), (255,255,255), thickness=2) # 引数　rectangle(画像データ、検出した物を囲う四角の左頂点の座標、検出した物を囲う四角の右下頂点の座標、四角の色、四角の厚さ)
    
        if len(facerect) > 0: #facerectに入った要素数が0以上の時(物体を認識できたとき)
            print("x座標" + str((rect[0]+rect[2])/2)) #検出した物体の中心のx座標
            print("y座標" + str((rect[1]+rect[3])/2)) #検出した物体の中心のy座標
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
    cv2.imshow("Camera",frame)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()