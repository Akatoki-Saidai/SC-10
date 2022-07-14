import cv2
import numpy as np
cascade = cv2.CascadeClassifier("C:\\Users\\konan\\Desktop\\haarcascade_frontalface_alt.xml")#用いるデータ(カスケード分類器)のパス
cap = cv2.VideoCapture(0)#カメラの映像取得

#赤色検知を行う関数
def red_detect(img):
    # HSV色空間に変換(ここでは、取得した映像を処理するために、RGBからHSVに変更するのと、赤色と認識するHSVの値域を設定してます。RGBからHSVにするのは、人間が見る色の感覚とと近いから)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1(OpenCVのHSVの範囲は普通の範囲と違うので、ここではOpenCVで赤とされる0～30 150～179が色相としてます）[色相,彩度,明度]
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
    #違う関数で、変数を使いたいから、グローバルに。
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

                return maxblob #取得したブロブの情報を返してあげます

    
    raise ValueError



#赤色認識を行う関数とブロブ解析を行う関数を実行し、結果表示する関数
def main():
    #保存する動画のfps
    fps = 30

    # 録画する動画のフレームサイズ
    size = (640, 480)

    # 出力する動画ファイルの設定（映像を取得して処理するスピードとfpsがかみ合わないと、保存した映像が早送りになってしまう。安いラズパイ用のカメラだと12fpsくらいじゃないと動画が早送りになる。)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video = cv2.VideoWriter('output.avi', fourcc, fps, size)
    while(cap.isOpened()):
        # フレームを取得
        ret, frame = cap.read()#capに入れられた、カメラの映像を入れる。 retは映像を取得できたかどうかをbool型でいれられる

        # 赤色検出(取得できた映像を赤色検出の関数にまわす)
        mask = red_detect(frame)
        

        try:


            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            target = analysis_blob(mask)
            #最初にカメラが起動した際、物体の面積が40000未満ならrecognition関数を実行
            if (data[:, 4][max_index] < 40000): 
                recognition()
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
        if (data[:, 4][max_index] >30000):
            print("hello")

        # qキーが押されたら途中終了
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()



#学習したデータで物体を区別するプログラム
def recognition():
    while True:

        ret,frame = cap.read()
        facerect = cascade.detectMultiScale(frame)
        mask = red_detect(frame)
        try:


            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            #recognition関数が実行された際に、そこでmain()の処理が止まり、面積が取得できなくなるので、recognitionにも赤色検知とブロブ解析を行う文を書くことで、面積を取得している
            target = analysis_blob(mask)
        except ValueError:
            continue


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


        #"w"を押すか、認識できなくなるか、面積が50000を超えると停止(40000未満になれば、また実行される)
        #なぜかcv2.waitKey(25) & 0xFF == ord('w')を消すとうまく動作しなくなる
        if (cv2.waitKey(25) & 0xFF == ord('w') or  data[:, 4][max_index] >50000  or len(facerect) == 0):
            break

#実行
if __name__ == '__main__':
    main()