# -*- coding: utf-8 -*-
"""
Created on Mon Feb 25 22:54:45 2019

@author: 幸雄
"""
#####
#https://future-tech-association.org/2017/12/05/ai_by_opencv/
#https://future-tech-association.org/2018/02/10/ai_by_opencv2/
import cv2
#import picamera
import numpy as np


#現物の画像サンプルが手元にないので、パラメータは適当です。

#画像を平滑化する関数
def blur(img):
    blurred_img = cv2.GaussianBlur(img, (17, 17), 0)
    blurred_img = cv2.GaussianBlur(blurred_img, (55, 55), 0)
    return blurred_img

#2値化する関数
def binarize(img):
    thresh = 120 #閾値
    maxValue = 255 #非0のピクセルをこの値に置き換える
    ret,binarized_img = cv2.threshold(img,thresh,maxValue,cv2.THRESH_BINARY)
    #merged = cv2.bitwise_not(merged)#白黒反転。柿ピー画像は黒地に白になるはずなので、不要。
    return binarized_img

#2値化画像のノイズを取り除く関数
def morph(img):
    kernel = np.ones((3, 3),np.uint8)
    morphed_img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel, iterations=2)
    morphed_img = cv2.morphologyEx(morphed_img, cv2.MORPH_CLOSE, kernel)
    #morphed_img = medianBlur(img, 5)#メディアンフィルタ
    return morphed_img


def resize(img, scaling):
    height = img.shape[0]
    width = img.shape[1]
    resized_img = cv2.resize(img,(int (width * scaling), int (height * scaling)))
    return resized_img




font = cv2.FONT_HERSHEY_SIMPLEX
fontSize = 0.3
fontColor = (255,0,0)
fontThickness = 1
fontLine = cv2.LINE_AA

#カメラ設定（ラズパイ用カメラモジュールを使う場合はOpenCVではなくpicameraライブラリを使用する）
cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)


while True:
    ret,img = cap.read()
    roi = img[10:220, 10:220]#ROIを設定
    grayed = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)#grayed = (cv2.split(img))[0]

    img_blurred = blur(grayed)#平滑化
    img_binarized = binarize(img_blurred)#2値化
    img_binarized = morph(img_binarized)#ゴミ除去

    cv2.imshow('captured image', roi) 
    cv2.imshow('bin image', img_binarized)

    image, contours, hierarchy = cv2.findContours(img_binarized, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#輪郭検出
    contoured = cv2.drawContours(roi, contours, -1, (0, 0, 255), 1)#輪郭線を描画

    for i, c in enumerate(contours):#オブジェクトごとに処理
        M = cv2.moments(c)#モーメントを計算
        x_g = int(M['m10']/M['m00'])#重心を計算
        y_g = int(M['m01']/M['m00'])
        cv2.circle(contoured, (x_g, y_g), 3, (255,0,0), -1)#重心点を描画
        cv2.putText(contoured, ('(%d,%d)' %(x_g,y_g)),(x_g + 4,y_g),font,fontSize,fontColor,fontThickness,fontLine)#重心座標を描画

        #輪郭線の外接する矩形領域Rを抽出
        x_min = min((c[:,0])[:,0])
        x_max = max((c[:,0])[:,0])
        y_min = min((c[:,0])[:,1])
        y_max = max((c[:,0])[:,1])
        R_bin = img_binarized[x_min:x_max, y_min:y_max]
        R_img = grayed[x_min:x_max, y_min:y_max]
        contoured = cv2.rectangle(contoured, (x_min, y_min), (x_max, y_max), (0, 255, 0), 1)#矩形描画

        #ToDo:柿ピーかどうかを判定する処理をここに記述
        mean_val = cv2.mean(R_img, mask = R_bin)
        #if value > xxx:
            #柿
        #else:
            #ピー
        cv2.putText(contoured, ('mean:%d' %mean_val[0]),(x_g,y_g + 12),font,fontSize,fontColor,fontThickness,fontLine)#平均画素値を描画
    
    #img = resize(contoured)
    cv2.imshow('object detection image', contoured)
    key = cv2.waitKey(10)
    if key == 27:  # ESCキーで終了
        break

cap.release()
cv2.destroyAllWindows()