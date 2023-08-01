import serial, time, csv
import datetime

import numpy as np

num = 13

tof = [0]*num
stm32f7 = serial.Serial('COM3',115200,timeout=100)

print("Starting Conversation with Arduino")
print(len(tof))
SayingTo = input("Enter a message to send: ")

stm32f7.write((SayingTo + '\n').encode())

print(SayingTo)
time.sleep(1)

now = datetime.datetime.now()  # 현재 날짜와 시간을 얻습니다.
time_string = now.strftime("%y%m%d_%H%M")  # 날짜와 시간을 원하는 형식의 문자열로 변환합니다.

while True:

    while (stm32f7.inWaiting()==0):
        pass
    SdataPacket = stm32f7.readline()
    SdataPacket = str(SdataPacket, "utf-8")
    SsplitPacket = SdataPacket.split(" ")



    # 이 문자열을 파일 경로에 포함시킵니다.
    file_path = "C:/Users/21310/Desktop/PJH/Software/TACTS/MicroController/DataCol_Python/CollectedData/" + time_string + "_Data.csv"
    #print(file_path)

    if len(SsplitPacket) >= len(tof):
        file = open(file_path, 'a',encoding = "utf-8", newline='')
        csv_writer = csv.writer(file)
        csv_writer.writerow(SsplitPacket)
        for i in range(0,num):
            tof[i] = float(SsplitPacket[i])
            print(tof[i],end=' ')
    print("")