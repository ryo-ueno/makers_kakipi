# -*- coding: utf-8 -*-
"""
Created on Mon Feb 25 22:54:45 2019

@author: 幸雄
"""
#####
#https://future-tech-association.org/2017/12/05/ai_by_opencv/
#https://future-tech-association.org/2018/02/10/ai_by_opencv2/
import cv2
#import numpy as np


#画像を平滑化する関数
def morph_and_blur(img):
    #kernel = np.ones((3, 3),np.uint8)
    m = cv2.GaussianBlur(img, (17, 17), 0)
    #m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, iterations=2)
    m = cv2.GaussianBlur(m, (55, 55), 0)
    return m

def binary_threshold_for_birds(img):
    #upper_thresh = 120 #この値より明るい画素(背景)を白にするパラメータ
    #under_thresh = 160 #この値より明るい画素を黒で強調する(輪郭強調の)パラメータ
    #maxValue = 255
    #th, drop_back = cv2.threshold(grayed, upper_thresh, maxValue, cv2.THRESH_BINARY)
    #th, clarify_born = cv2.threshold(grayed, under_thresh, maxValue, cv2.THRESH_BINARY_INV)
    #merged = np.minimum(drop_back, cv2.bitwise_not(clarify_born))
    merged = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,501,50)
    merged = cv2.bitwise_not(merged)
    return merged

def resize(img):
    height = img.shape[0]
    width = img.shape[1]
    img2 = cv2.resize(img,(int (width/1), int (height/1)))
    return img2


cap = cv2.VideoCapture(0)
cap.set(4,1280) 
cap.set(5,720) 
cap.set(6,6) 

while True:
    ret,img = cap.read()
#    grayed = (cv2.split(img))[0]#Blueチャンネルを取り出す
    grayed = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img_morphed = morph_and_blur(grayed)

    img_morphed_resize = resize(img_morphed)    
    cv2.imshow('video image1', img_morphed_resize)

    img_bin = binary_threshold_for_birds(img_morphed)
    image, contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contoured = cv2.drawContours(img, contours, -1, (0, 0, 255), 1)
    
    img_bin = resize(img_bin)
    cv2.imshow('video image2', img_bin)
    key = cv2.waitKey(10)
    if key == 27:  # ESCキーで終了
        break

    for c in contours:
        x_min = min((c[:,0])[:,0])
        x_max = max((c[:,0])[:,0])
        y_min = min((c[:,0])[:,1])
        y_max = max((c[:,0])[:,1])
        contoured = cv2.rectangle(contoured, (x_min, y_min), (x_max, y_max), (0, 255, 0), 1)
#    height = contoured.shape[0]
#    width = contoured.shape[1]
#    img = cv2.resize(contoured,(int (width/1.5), int (height/1.5)))
    
    img = resize(contoured)
    cv2.imshow('video image3', img)
    key = cv2.waitKey(10)
    if key == 27:  # ESCキーで終了
        break

cap.release()
cv2.destroyAllWindows()