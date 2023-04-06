from vpython import *
from time import *
import numpy as np
import math
import serial

port = 'COM3' # 시리얼 포트
baud = 115200 # 시리얼 보드레이트(통신속도)

ad=serial.Serial(port,baud)
sleep(1)


toRad = 2*np.pi/360
toDeg = 1/toRad

#canvas
scene.range = 200
scene.forward = vector(-1,-1,-1)
scene.width = 480
scene.height = 640
scene.background = color.white
scene.up = vector(0,0,1)


# xzy axis
axis_length = 40
axis_width = 4
xarrow = arrow(length=axis_length,shaftwidth=axis_width,color=color.red, axis=vector(1,0,0))
zarrow = arrow(length=axis_length,shaftwidth=axis_width,color=color.green, axis=vector(0,0,1))
yarrow = arrow(length=axis_length,shaftwidth=axis_width,color=color.blue, axis=vector(0,1,0))

D = 160 #cylinder
R = D/2
L = 216 #cylinder

sensor_cylinder = cylinder(axis=vector(0,0,1),pos=vector(0,0,0),radius=R,length=L,opacity=.3)


DisSenLen = 15
DisSenWid = 10
DisSenHeight = 2 # thickness

for i in range(0,12):
    DisSenR = 50
    DisSenTheta[i] = 30 * i
    DisSenX = DisSenR*cos(DisSenTheta[i]*toRad)
    DisSenY = DisSenR*sin(DisSenTheta[i]*toRad)
    DisSenZ = 105

    DisSen = box(size = vector(DisSenLen,DisSenHeight,DisSenWid), pos = vector(DisSenX,DisSenY,DisSenZ),up = vector(DisSenX,DisSenY,0),color = vector(.62,0,.63))


    ball = sphere(radius = 2,pos = vector(ballX,ballY,DisSenZ),color= vector(1,0,0))





while (True):
    while (ad.inWaiting()==0):
        pass
    dataPacket = ad.readline()
    dataPacket = str(dataPacket,"utf-8")
    splitPacket = dataPacket.split(" ")



    # input variable
    for i in range(0,12):
        ballR[i] = splitPacket[i]
        ballR[i] = 30+DisSenR
        ballX = ballR*cos(DisSenTheta[i]*toRad)
        ballY = ballR*sin(DisSenTheta[i]*toRad)
        ballz = DisSenZ

    print("theta0=",theta0,"z=",z)


    rate(100)