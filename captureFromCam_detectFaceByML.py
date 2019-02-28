

import numpy as np
import cv2
import getFps

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

gFrameRate = getFps.FrameRate()

cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)

font = cv2.FONT_HERSHEY_SIMPLEX
fontSize = 0.4
fontColor = (255,255,255)
fontThickness = 1
fontLine = cv2.LINE_AA

while(True):
    #Capture frame-by-frame
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    fps = gFrameRate.get()
    h, w = frame.shape[:2]
    cv2.putText(frame, ('Size: %dx%d' %(w,h)),(10,15),font,fontSize,fontColor,fontThickness,fontLine)
    cv2.putText(frame, ('FrameRate: %dfps' %fps),(10,30),font,fontSize,fontColor,fontThickness,fontLine)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    for (x,y,w,h) in faces:
        frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = frame[y:y+h, x:x+w]
        eyes = eye_cascade.detectMultiScale(roi_gray)
        for (ex,ey,ew,eh) in eyes:
            cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
    
    cv2.imshow('frame', frame) 
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()