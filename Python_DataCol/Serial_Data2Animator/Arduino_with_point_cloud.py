import serial, time, csv
from vpython import *
import numpy as np


tof = [0,0,0,0,0,0,0,0,0,0,0,0,0,0]
arduino = serial.Serial('COM4',115200,timeout=1)

print("Starting Conversation with Arduino")


SayingTo = input()

SayingToArduino = SayingTo.encode("utf-8")
arduino.write(SayingToArduino)
print(SayingToArduino)
time.sleep(1)


DisSenTheta=[0,0,0,0,0,0,0,0,0,0,0,0]
ball=[0,0,0,0,0,0,0,0,0,0,0,0]


#Math
toRad = 2*np.pi/360
toDeg = 1/toRad

#canvas
scene.range = 200
scene.forward = vector(-1,-1,-1)
scene.width = 480
scene.height = 640
scene.background = color.white
scene.up = vector(0,0,1)


# xyz axis
AxisLen = 40
AxisWid = 4
xArrow = arrow(length=AxisLen,shaftwidth=AxisWid,color=color.red, axis=vector(1,0,0))
yArrow = arrow(length=AxisLen,shaftwidth=AxisWid,color=color.blue, axis=vector(0,1,0))
zArrow = arrow(length=AxisLen,shaftwidth=AxisWid,color=color.green, axis=vector(0,0,1))

R = 80  #cylinder D=160
L = 216 #cylinder

sensor_cylinder = cylinder(axis=vector(0,0,1),pos=vector(0,0,0),radius=R,length=L,opacity=.3)

DisSenLen = 15
DisSenWid = 10
DisSenHeight = 2 # thickness






for i in range(0,10):
    DisSenR = 50
    DisSenTheta[i] = 36 * i
    DisSenX = DisSenR*cos(DisSenTheta[i]*toRad)
    DisSenY = DisSenR*sin(DisSenTheta[i]*toRad)
    DisSenZ = 105

    ballR = 30 + DisSenR
    ballX = ballR * cos(DisSenTheta[i] * toRad)
    ballY = ballR * sin(DisSenTheta[i] * toRad)
    ballz = DisSenZ

    DisSen = box(size = vector(DisSenLen,DisSenHeight,DisSenWid), pos = vector(DisSenX,DisSenY,DisSenZ),up = vector(DisSenX,DisSenY,0),color = vector(.62,0,.63))
    ball[i] = sphere(radius=2, pos=vector(ballX, ballY, DisSenZ), color=vector(1, 0, 0))




while True:

    while (arduino.inWaiting()==0):
        pass
    SdataPacket = arduino.readline()
    SdataPacket = str(SdataPacket, "utf-8")
    SsplitPacket = SdataPacket.split(" ")

    file_path = "C:/Users/Lab/Desktop/LYS/Coding/Python/0805_70ms_Mean3ea.csv"
    if len(SsplitPacket) >= len(tof):
        file = open(file_path, 'a',encoding = "utf-8", newline='')
        csv_writer = csv.writer(file)
        csv_writer.writerow(SsplitPacket)
        for i in range(0,14):
            tof[i] = float(SsplitPacket[i])
            print(tof[i],end=' ')
        for i in range(0,10):
            tof[i] = float(SsplitPacket[i])
            ball[i].pos.x= (tof[i]+DisSenR)*cos(DisSenTheta[i]*toRad)
            ball[i].pos.y= (tof[i]+DisSenR)*sin(DisSenTheta[i]*toRad)
    print("")