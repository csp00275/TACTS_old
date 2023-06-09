import serial
import numpy as np

simple = serial.Serial('COM4',115200,timeout=1)

tof = [0,0,0,0,0,0,0,0,0,0]

while (True):
    while (simple.inWaiting()==0):
        pass
    SdataPacket = simple.readline()
    SdataPacket = str(SdataPacket, "utf-8")
    SsplitPacket = SdataPacket.split(" ")

    for i in range(0,10):
        tof[i] = float(SsplitPacket[i])
        print(tof[i],end=' ')
    print("")
