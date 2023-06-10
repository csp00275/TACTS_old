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

conArrowXDir=0
conArrowYDir=0
conArrowXPos=0
conArrowYPos=0
conArrowLen=80


ConArrow0 = arrow(length=conArrowLen, shaftwidth=5, color=color.yellow)

while (True):
    while (ad.inWaiting()==0):
        pass
    dataPacket = ad.readline()
    dataPacket = str(dataPacket,"utf-8")
    splitPacket = dataPacket.split(" ")



    # input variable
    r = R - 10
    theta0 = float(splitPacket[0]) * 2
    z = float(splitPacket[1])

    print("theta0=",theta0,"z=",z)
    costheta = cos(toRad * theta0)
    sintheta = sin(toRad * theta0)

    conArrowXDir = r * costheta
    conArrowYDir = r * sintheta

    conArrowLen = 80

    conArrowXPos = (R + conArrowLen) * costheta
    conArrowYPos = (R + conArrowLen) * sintheta
    ConArrow0.axis = vector(-conArrowXDir, -conArrowYDir, 0)
    ConArrow0.pos = vector(conArrowXPos, conArrowYPos, z)
    rate(100)