# -*- coding: utf-8 -*-

import cv2
import numpy as np
import copy
import serial
import picamera
import picamera.array
import RPi.GPIO as GPIO
import time

#LED初期設定
# GPIO.BOARD: PIN番号
# GPIO.BCM:   GPIO番号
LedRed =   11
LedGreen = 13
LedBlue =  12
LedWhite = 5
GPIO.setmode(GPIO.BOARD)
GPIO.setup( LedRed,   GPIO.OUT )
GPIO.setup( LedGreen, GPIO.OUT )
GPIO.setup( LedBlue,  GPIO.OUT )
GPIO.setup( LedWhite, GPIO.OUT )
#各種パラメータ
LED_LIGHT = False#LED点灯有無
DISPLAY = True#画面表示の有無
ROI_RANGE = {"x":10, "y":64, "w":300, "h":126}
GAUSSIAN_KERNEL = (11, 11)#平滑（ガウシアン）フィルタのカーネルサイズ。大きいほどぼやける。
BIN_THRESHOLD = 70#2値化閾値
MOLPHOLOGY_KERNEL = np.ones((5, 5),np.uint8)#ゴミ除去（収縮・膨張）処理のカーネル。
JUDGE_THRESHOLD = 64#柿かピーかの判定閾値
DISPLACEMENT_THRESHOLD = 20#フレーム間の同一物体判定するときの、座標の誤差許容範囲[pix]
VELOCITY = 57 #搬送速度[mm/s]
PIX_PER_MM = 24 #1mmは何pixか[pix/mm]
BELT_INTERVAL = 100 #カメラと分配部との距離[mm]。タイミング微調整する時はこれを変えましょう。
IGNORE_AREA = 5 #[mm]。画像端のこの領域にある（＝見切れてる）柿ピーは処理から除外する。
#カメラパ設定
CAP_WIDTH = 320#カメラ画像取込幅
CAP_HEIGHT = 240#カメラ画像取込高さ
EXPOSURE_MODE = 'night'#カメラ露光設定 もしくは’off’
ROTATION = 0#画像の回転90度刻み
AWB_MODE = 'shade'#ホワイトバランス
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
            #send = bytes(judge[n] + ' ' + str(wait[n]) + ';', 'utf-8')
            #ser.write(send)
            #recieved.append(ser.readline())#エコーされた文字列を受信
    return received


#変数の初期化
g_prev = []
judge_prev = []
time_prev = 0

#カメラ設定（USB Web Cameraの場合）
#cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FPS, 30)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_WIDTH)

#カメラ設定（ラズパイ用カメラモジュールの場合）
camera = picamera.PiCamera()
camera.resolution = (CAP_WIDTH, CAP_HEIGHT)
camera.exposure_mode = EXPOSURE_MODE
camera.rotation = ROTATION
camera.awb_mode = AWB_MODE

#LED
GPIO.output(LedRed,LED_LIGHT)
GPIO.output(LedGreen,LED_LIGHT)
GPIO.output(LedBlue,LED_LIGHT)
GPIO.output(LedWhite,LED_LIGHT)


while True:
    #変数の初期化
    g = []#今回のフレームのオブジェクト座標リスト
    judge = []#今回のフレームの柿ピー判定リスト
    index = []#今回のフレームで新たに出現したオブジェクトのインデックスリスト
    wait = []#arduinoが命令を受け取ってから分配部を動作させるまでのインターバルタイム

    #カメラから画像を取得（USB Web Cameraの場合）
    #ret,img = cap.read()

    #カメラから画像を取得（ラズパイ用カメラモジュールの場合）
    img = None
    with picamera.array.PiRGBArray(camera, size = (CAP_WIDTH, CAP_HEIGHT)) as stream:
        camera.capture(stream, 'bgr', use_video_port=True)
        img = stream.array

    #時間を記録
    time_now = cv2.getTickCount()/cv2.getTickFrequency()#時刻を取得[s]
    dt = time_now - time_prev#前フレームからの経過時間[s]

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
        if int(M['m01']/M['m00']) < IGNORE_AREA * PIX_PER_MM  and int(M['m01']/M['m00']) > ROI_RANGE["w"] - IGNORE_AREA * PIX_PER_MM:
            continue#ROIから見切れている柿ピーは除外(面積M['m00']で除外するのもアリ？)
        g.append((int(M['m10']/M['m00']),int(M['m01']/M['m00'])))#重心(x,y)をリストgに追加

        #輪郭線の外接する矩形領域を抽出
        x,y,w,h = cv2.boundingRect(c)
        R_img = blueImg[y:y+h, x:x+w]
        R_bin = cv2.drawContours(np.zeros((h, w),np.uint8), [c[:,0]-(x,y)], 0, 255, -1)#オブジェクトのマスク。R_bin = img_binarized[y:y+h, x:x+w]だと矩形内に複数物体があるとアウト。

        #柿かピーかを判定する
        mean_val = cv2.mean(R_img, mask = R_bin)
        if mean_val[0] < JUDGE_THRESHOLD:
            judge.append('K')#柿
        else:
            judge.append('P')#ピー

        #前回のフレームと今回のフレームのオブジェクトを判別。同一オブジェクトに対して2回以上分別命令を送らないように。
        flag = True
        for k, jp in enumerate(judge_prev):
            if judge[i] == jp and \
                abs(g[i][1] - g_prev[k][1] + VELOCITY * PIX_PER_MM) < DISPLACEMENT_THRESHOLD and \
                abs(g[i][0] - g_prev[k][0]) < DISPLACEMENT_THRESHOLD:#画面右から左に(y負方向）柿ピーが流れていくとする。
                #ToDo:もしも分別タイミングの制御がシビアなら、速度(-g[i][1]+g_prev[j][1])/dtから分配部到達予想時間を求めるなどの工夫が必要
                flag = False
                break
        if flag:
            index.append(i)
            intervalTime = (BELT_INTERVAL + g[i][1] / PIX_PER_MM) / VELOCITY#分配部への到達見込み時間。
            wait.append(intervalTime)

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
    time_prev = time_now

    if cv2.waitKey(1) == 27:  # ESCキーで終了
        break

#カメラをリリース（USB Web Cameraの場合）
#cap.release()

#カメラをリリース（ラズパイ用カメラモジュールの場合）
camera.close()

#LEDをOFF
GPIO.output(LedRed,False)
GPIO.output(LedGreen,False)
GPIO.output(LedBlue,False)
GPIO.output(LedWhite,False)

cv2.destroyAllWindows()