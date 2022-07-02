 # -*- coding: utf-8 -*-
import cv2
import numpy as np



def red_detect(img):
    # HSV色空間に変換
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0, 127, 0])
    hsv_max = np.array([30, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([150, 127, 0])
    hsv_max = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

    return mask1 + mask2

# ブロブ解析


def analysis_blob(binary_img):
    global label
    global data
    global max_index
    global center
    # 2値画像のラベリング処理
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
                maxblob["area"] = data[:, 4][max_index]   # 面積
                maxblob["center"] = center[max_index]  # 中心座標

                return maxblob

    
    raise ValueError


def main():
   
    # カメラのキャプチャ
    cap = cv2.VideoCapture(0)
     
    fps = 30

    # 録画する動画のフレームサイズ（webカメラと同じにする）
    size = (640, 480)

    # 出力する動画ファイルの設定
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video = cv2.VideoWriter('output.avi', fourcc, fps, size)

    while(cap.isOpened()):
        # フレームを取得
        ret, frame = cap.read()

        # 赤色検出
        mask = red_detect(frame)

        
        try:

            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            target = analysis_blob(mask)

            # 面積最大ブロブの中心座標を取得
            center_x = int(target["center"][0])
            center_y = int(target["center"][1])

            # フレームに面積最大ブロブの中心周囲を円で描く
            cv2.circle(frame, (center_x, center_y), 50, (0, 200, 0),
                       thickness=3, lineType=cv2.LINE_AA)

            
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
        #カメラの映像は横640にしてるので、320付近がカメラの中心です
        if  270 <= center[max_index][0] and center[max_index][0] < 370 and  50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
            #まっすぐ進み動作(物体が中心近くにいる際)
            print("まっすぐ")
        
        elif data[:, 4][max_index] <= 50:
            #回転動作("赤い物体を検出できなくなった際")
            print("回転")
        
        elif center[max_index][0] < 270 and  50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
            #回転する動作(物体がカメラの中心から左にずれている際)
            print("左回転")
        
        elif center[max_index][0] >= 370 and  50 < data[:, 4][max_index] and data[:, 4][max_index] <= 80000:
            #回転する動作（物体がカメラの中心から右にずれている際）
            print("右回転")

        elif data[:, 4][max_index] > 80000:
            #止まる(物体の近くに接近した際)
            print("止まる")       
            　 # qキーが押されたら途中終了(何かあった時ように、手動停止)
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    cap.release()
    video.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
