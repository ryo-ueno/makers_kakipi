# -*- coding: utf-8 -*-
"""
Created on Mon Feb 25 22:54:45 2019

@author: 幸雄
"""
#####
#https://future-tech-association.org/2017/12/05/ai_by_opencv/
#https://future-tech-association.org/2018/02/10/ai_by_opencv2/
import cv2
import numpy as np


#画像を平滑化する関数
def morph_and_blur(img):
    #kernel = np.ones((3, 3),np.uint8)
    m = cv2.GaussianBlur(img, (17, 17), 0)
    #m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, iterations=2)
    #m = cv2.GaussianBlur(m, (5, 5), 0)
    return m

def binary_threshold_for_birds(img):
    #upper_thresh = 120 #この値より明るい画素(背景)を白にするパラメータ
    #under_thresh = 160 #この値より明るい画素を黒で強調する(輪郭強調の)パラメータ
    #maxValue = 255
    #th, drop_back = cv2.threshold(grayed, upper_thresh, maxValue, cv2.THRESH_BINARY)
    #th, clarify_born = cv2.threshold(grayed, under_thresh, maxValue, cv2.THRESH_BINARY_INV)
    #merged = np.minimum(drop_back, cv2.bitwise_not(clarify_born))
    merged = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,21,6)
    merged = cv2.bitwise_not(merged)
    return merged




path = 'birds.png'#'C:/sutudy/img/birds.png'

img = cv2.imread(path) #path は画像ファイルのパス。
grayed = (cv2.split(img))[0]#Blueチャンネルを取り出す

cv2.imshow('window1',grayed)
cv2.waitKey(1000)
cv2.destroyAllWindows()

img_morphed = morph_and_blur(grayed)
img_bin = binary_threshold_for_birds(img_morphed)

cv2.imshow('window2',img_bin)
cv2.waitKey(1000)
cv2.destroyAllWindows()


image, contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#print(contours)

#i=0
#for c in contours:
#    epsilon = cv2.arcLength(c,True)
#    print(epsilon)
#        arclen = cv2.arcLength(np.array(contours[i]),True) 
#    print('contours',i,c)
    
#    epsilon = 0.1*cv2.arcLength(c[i],True)
#    approx = cv2.approxPolyDP(c[i], epsilon, True)

    
#    i += 1
#    if i > 10:
#        break
    
#for c in contours:#[0]:
contoured = cv2.drawContours(img, contours, -1, (0, 0, 255), 1)

for c in contours:
    x_min = min((c[:,0])[:,0])
    x_max = max((c[:,0])[:,0])
    y_min = min((c[:,0])[:,1])
    y_max = max((c[:,0])[:,1])
    contoured = cv2.rectangle(contoured, (x_min, y_min), (x_max, y_max), (0, 255, 0), 1)

cv2.imshow('window3',contoured)
cv2.waitKey(10000)
cv2.destroyAllWindows()