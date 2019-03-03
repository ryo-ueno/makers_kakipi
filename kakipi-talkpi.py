# -*- coding:utf-8 -*-  
 
import os
from datetime import datetime
import serial
import time

def hear_kakitype():
    #シリアル値を読み込んで指定時間停止する
    ser = serial.Serial('/dev/ttyACM0', 9600)
    time.sleep(0.5)
    t0 = time.time()

    while time.time() - t0 < 1:
        # シリアル通信でデータを受信
        str = ser.read(4)
        # 読み込んだデータの表示
        #print(str.decode())
    return(str)
    
def say_kakitype(hear_type):
#    print("A"+hear_type)
#    print('0' in hear_type)
    if '1' in hear_type:
        text = '柿の種だよ' 
        print(text+"!")
        return(text)
    else:
        text = 'ピーナッツだよ' 
        print(text+"!!")
        return(text)
 
### main
if __name__ == "__main__":
    kakitype = hear_kakitype().decode()
#    print("D"+kakitype)
    type_talk = say_kakitype(kakitype)
    cmd = "./python/aquestalkpi/AquesTalkPi " + type_talk + " -s 90 | aplay"
    os.popen(cmd).readline().strip()
