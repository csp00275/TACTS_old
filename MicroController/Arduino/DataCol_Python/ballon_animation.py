from vpython import *
from time import *
import numpy as np
import math
DisSenTheta=[0,0,0,0,0,0,0,0,0,0,0,0]

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

for i in range(0,12):
    DisSenR = 50
    DisSenTheta[i] = 30 * i
    DisSenX = DisSenR*cos(DisSenTheta[i]*toRad)
    DisSenY = DisSenR*sin(DisSenTheta[i]*toRad)
    DisSenZ = 105

    #DisSen = box(size = vector(DisSenLen,DisSenHeight,DisSenWid), pos = vector(DisSenX,DisSenY,DisSenZ),up = vector(DisSenX,DisSenY,0),color = vector(.62,0,.63))

    ballR = 30+DisSenR
    ballX = ballR*cos(DisSenTheta[i]*toRad)
    ballY = ballR*sin(DisSenTheta[i]*toRad)
    ballz = DisSenZ
    ball = sphere(radius = 2,pos = vector(ballX,ballY,DisSenZ),color= vector(1,0,0))

while (True):
    ballR = 30 + DisSenR
    rate(100)

