from vpython import *
from time import *
import numpy as np
import math
DisSenTheta=[0,0,0,0,0,0,0,0,0,0,0,0]

toRad = 2*np.pi/360
toDeg = 1/toRad

#canvas
scene.range = 200 #보이는 화면 크기
scene.forward = vector(-1,-1,-1) #처음에 보이는 화면 방향
scene.width = 480 #보이는 배경 폭
scene.height = 640 #높이
scene.background = color.white
scene.up = vector(0,0,1) #위 방향


# xyz axis
AxisLen = 40
AxisWid = 4
xArrow = arrow(length=AxisLen,shaftwidth=AxisWid,color=color.red, axis=vector(1,0,0))
yArrow = arrow(length=AxisLen,shaftwidth=AxisWid,color=color.blue, axis=vector(0,1,0))
zArrow = arrow(length=AxisLen,shaftwidth=AxisWid,color=color.green, axis=vector(0,0,1))

R = 80  #cylinder D=160
L = 216 #cylinder Length

sensor_cylinder = cylinder(axis=vector(0,0,1),pos=vector(0,0,0),radius=R,length=L,opacity=.3)


#센서
DisSenLen = 15
DisSenWid = 10
DisSenHeight = 2 # thickness

for i in range(0,12):
    DisSenR = 50 #센서 반지름
    DisSenTheta[i] = 30 * i
    DisSenX = DisSenR*cos(DisSenTheta[i]*toRad)
    DisSenY = DisSenR*sin(DisSenTheta[i]*toRad)
    DisSenZ = 105

    #DisSen = box(size = vector(DisSenLen,DisSenHeight,DisSenWid), pos = vector(DisSenX,DisSenY,DisSenZ),up = vector(DisSenX,DisSenY,0),color = vector(.62,0,.63))

    ballR = 30+DisSenR
    ballX = ballR*cos(DisSenTheta[i]*toRad)
    ballY = ballR*sin(DisSenTheta[i]*toRad) #센서랑 같은 위치 반지름만 30+
    ballz = DisSenZ
    ball = sphere(radius = 2,pos = vector(ballX,ballY,DisSenZ),color= vector(1,0,0))

while (True):
    ballR = 30 + DisSenR
    rate(100)

