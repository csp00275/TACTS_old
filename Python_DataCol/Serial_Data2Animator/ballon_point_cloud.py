from vpython import *
from time import *
import numpy as np
import math
import serial

simple = serial.Serial('COM4',115200,timeout=1)

tof = [0,0,0,0,0,0,0,0,0,0]
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

R = 100  #cylinder D=160
L = 200 #cylinder

sensor_cylinder = cylinder(axis=vector(0,0,1),pos=vector(0,0,0),radius=R,length=L,opacity=.3)

#센서
DisSenLen = 15
DisSenWid = 10
DisSenHeight = 2 # thickness

for i in range(0,10):
    DisSenR = 50
    DisSenTheta[i] = 36 * i +90
    DisSenX = DisSenR*cos(DisSenTheta[i]*toRad)
    DisSenY = DisSenR*sin(DisSenTheta[i]*toRad)
    DisSenZ = 105

    ballR = 50 + DisSenR
    ballX = ballR * cos(DisSenTheta[i] * toRad)
    ballY = ballR * sin(DisSenTheta[i] * toRad)
    ballz = DisSenZ

    DisSen = box(size = vector(DisSenLen,DisSenHeight,DisSenWid), pos = vector(DisSenX,DisSenY,DisSenZ),up = vector(DisSenX,DisSenY,0),color = vector(.62,0,.63))
    ball[i] = sphere(radius=2, pos=vector(ballX, ballY, DisSenZ), color=vector(1, 0, 0))





while (True):
    while (simple.inWaiting()==0):
        pass
    SdataPacket = simple.readline()
    SdataPacket = str(SdataPacket, "utf-8")
    SsplitPacket = SdataPacket.split(" ")
    if len(SsplitPacket) > len(tof):
        for i in range(0,10):
            tof[i] = float(SsplitPacket[i])
            ball[i].pos.x= (tof[i]+DisSenR)*cos(DisSenTheta[i]*toRad)
            ball[i].pos.y= (tof[i]+DisSenR)*sin(DisSenTheta[i]*toRad)
            print(tof[i],end=' ')
    print("")
    rate(100)



