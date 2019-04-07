# -*- coding: utf-8 -*-

import cv2
import numpy as np
import copy
#import serial
#import picamera


#各種パラメータ
DISPLAY = True#画面表示の有無
CAP_WIDTH = 320#カメラ画像取込幅
CAP_HEIGHT = 240#カメラ画像取込高さ
ROI_RANGE = {"x":10, "y":64, "w":300, "h":126}
GAUSSIAN_KERNEL = (11, 11)#平滑（ガウシアン）フィルタのカーネルサイズ。大きいほどぼやける。
BIN_THRESHOLD = 130#2値化閾値
MOLPHOLOGY_KERNEL = np.ones((5, 5),np.uint8)#ゴミ除去（収縮・膨張）処理のカーネル。
JUDGE_THRESHOLD = 64#柿かピーかの判定閾値
DISPLACEMENT_THRESHOLD = 20#フレーム間の同一物体判定するときの、座標の誤差許容範囲 pix
VELOCITY = 50 #想定される搬送速度 pix/frame
#フォント
FONT = cv2.FONT_HERSHEY_SIMPLEX
FSIZE = 0.3
FTHICKNESS = 1
FLINE = cv2.LINE_AA


#ToDo:arduinoに命令を送る関数
def send2arduino(judge, wait, index):
    received = []
    #with serial.Serial('/dev/ttyACM0',9600) as ser:
        #for n in index
            #send = bytes(judge[n] + ' ' + stf(wait[n]) + ';', 'utf-8')
            #ser.write(send)
            #recieved.append(ser.readline())#エコーされた文字列を受信
    return received

def resize(img, scaling):
    height = img.shape[0]
    width = img.shape[1]
    resized_img = cv2.resize(img,(int (width * scaling), int (height * scaling)))
    return resized_img


#変数の初期化
g_prev = []
judge_prev = []
time_prev = 0

#カメラ設定（USB Web Cameraの場合）
cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)

#カメラ設定（ラズパイ用カメラモジュールの場合）
#camera = picamera.PiCamera()
#stream = picamera.array.PiRGBArray(camera)
#camera.resolution = (CAP_WIDTH, CAP_HEIGHT)

while True:

    #変数の初期化
    g = []#今回のフレームのオブジェクト座標リスト
    judge = []#今回のフレームの柿ピー判定リスト
    index = []#今回のフレームで新たに出現したオブジェクトのインデックスリスト
    wait = []#arduinoが命令を受け取ってから分配部を動作させるまでのインターバルタイム

    #カメラから画像を取得（USB Web Cameraの場合）
    ret,img = cap.read()

    #カメラから画像を取得（ラズパイ用カメラモジュールの場合）
    #camera.capture(stream, 'bgr', use_video_port=True)
    #img = stream.array

    time = cv2.getTickCount()/cv2.getTickFrequency()#時刻を取得[s]
    dt = time - time_prev#前フレームからの経過時間[s]

    #画像の前処理
    roi = img[ROI_RANGE["y"]:ROI_RANGE["y"]+ROI_RANGE["h"], ROI_RANGE["x"]:ROI_RANGE["x"]+ROI_RANGE["w"]]#ROIを設定
    #grayed = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    blueImg, _, redImg = cv2.split(roi)#物体検出用⇒Redチャンネル、柿ピー判別⇒Blueチャンネル
    img_blurred = cv2.GaussianBlur(redImg, GAUSSIAN_KERNEL, 0)#平滑化
    ret, img_binarized = cv2.threshold(img_blurred,BIN_THRESHOLD,255,cv2.THRESH_BINARY)#2値化
    img_binarized = cv2.morphologyEx(img_binarized, cv2.MORPH_OPEN, MOLPHOLOGY_KERNEL)#ゴミ除去（オープニング）
    img_binarized = cv2.morphologyEx(img_binarized, cv2.MORPH_CLOSE, MOLPHOLOGY_KERNEL)#ゴミ除去（クロージング）
    #img_binarized = medianBlur(img_binarized, 5)#ゴミ除去（メディアンフィルタ）

    #輪郭検出
    image, contours, hierarchy = cv2.findContours(img_binarized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for i, c in enumerate(contours):#オブジェクトごとに処理

        #オブジェクトの座標を算出
        M = cv2.moments(c)#モーメントを計算
        g.append((int(M['m10']/M['m00']),int(M['m01']/M['m00'])))#重心(x,y)をリストgに追加

        #輪郭線の外接する矩形領域を抽出
        x,y,w,h = cv2.boundingRect(c)
        R_img = blueImg[y:y+h, x:x+w]
        R_bin = np.zeros((h, w),np.uint8)#オブジェクトのマスク。R_bin = img_binarized[y:y+h, x:x+w]だと矩形内に複数物体があるとアウト。
        cv2.drawContours(R_bin, [c[:,0]-(x,y)], 0, 255, -1)

        #柿かピーかを判定する
        #ToDo:ROIに見切れている柿ピーを除外すべき。（g[i][1]の範囲でスクリーニングする）
        #ToDo:面積でもスクリーニングをした方がよいか？
        mean_val = cv2.mean(R_img, mask = R_bin)
        if mean_val[0] < JUDGE_THRESHOLD:
            judge.append('K')#柿
        else:
            judge.append('P')#ピー

        #前回のフレームと今回のフレームのオブジェクトを判別。同一オブジェクトに対して2回以上分別命令を送らないように。
        flag = 0
        for k, jp in enumerate(judge_prev):
            #↓同一のオブジェクトとみなす条件式。画面右から左に(y負方向）柿ピーが流れていくとする
            if judge[i] == jp and abs(g[i][1] - g_prev[k][1] + VELOCITY) < DISPLACEMENT_THRESHOLD and abs(g[i][0] - g_prev[k][0]) < DISPLACEMENT_THRESHOLD:
                #ToDo:もしも分別タイミングの制御がシビアなら、速度(-g[i][1]+g_prev[j][1])/dtから分配部到達予想時間を求めるなどの工夫が必要
                flag = 1
                break
            else:
                pass
        if flag == 0:
            index.append(i)
            wait.append(500)#分配部への到達見込み時間。とりあえず固定値を入れときます。ToDo:g[i][1]とvelocityから時間を求める。

        #streamをリセット（ラズパイ用カメラモジュールの場合）
        #stream.seek(0)
        #stream.truncate()

        #デバッグ用描画処理
        if DISPLAY:
            cv2.circle(roi, (g[i][0], g[i][1]), 3, (255,0,0), -1)#重心点を描画
            cv2.putText(roi, ('(%d,%d)' %(g[i][0],g[i][1])),(g[i][0] + 4,g[i][1]),FONT,FSIZE,(255,0,0),FTHICKNESS,FLINE)#重心座標を描画
            cv2.drawContours(roi, [c], 0, (0, 0, 255), 1)#輪郭線を描画
            cv2.rectangle(roi, (x, y), (x+w, y+h), (0, 255, 0), 2)#矩形描画
            cv2.putText(roi, ('%s(mean:%d,S:%d)' %(judge[i],mean_val[0],M['m00'])),(g[i][0],g[i][1] + 12),FONT,FSIZE,(255,0,0),FTHICKNESS,FLINE)#判定結果、平均画素値、面積を描画

    #デバッグ用描画処理
    if DISPLAY:
        cv2.putText(img, ('%d fps' %(1/dt)),(0,10),FONT,FSIZE,(255,255,255),FTHICKNESS,FLINE)#フレームレートを描画
        cv2.rectangle(img, (ROI_RANGE["x"], ROI_RANGE["y"]), (ROI_RANGE["x"]+ROI_RANGE["w"], ROI_RANGE["y"]+ROI_RANGE["h"]), (255, 255, 255), 1)#ROI枠描画
        cv2.imshow('Image', img)
    
    #arduinoに命令を送る
    send2arduino(judge, wait, index)

    g_prev = copy.copy(g)
    judge_prev = copy.copy(judge)
    time_prev = time

    if cv2.waitKey(1) == 27:  # ESCキーで終了
        break

cap.release()
cv2.destroyAllWindows()