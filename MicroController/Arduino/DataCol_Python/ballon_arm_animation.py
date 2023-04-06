from vpython import *
from time import *
import numpy as np
import math
DisSenTheta=[0,0,0,0,0,0,0,0,0,0,0,0]

toRad = 2*np.pi/360
toDeg = 1/toRad

#canvas
scene.range = 400
scene.forward = vector(-1,-1,-1)
scene.width = 480
scene.height = 640
scene.background = color.black
scene.up = vector(0,0,1)


# xyz axis
AxisLen = 40
AxisWid = 4
xArrow = arrow(length=AxisLen,shaftwidth=AxisWid,color=color.red, axis=vector(1,0,0))
yArrow = arrow(length=AxisLen,shaftwidth=AxisWid,color=color.blue, axis=vector(0,1,0))
zArrow = arrow(length=AxisLen,shaftwidth=AxisWid,color=color.green, axis=vector(0,0,1))

R = 80  #cylinder D=160
L = 300 #cylinder

upper_arm = cylinder(axis=vector(0,0,1),pos=vector(0,0,0),radius=R,length=L,opacity=1,shininess =.1)
joint = sphere(axis=vector(0,0,1),pos=vector(0,0,0),radius=R,opacity=1)
fore_arm = cylinder(radius=R,length=L,opacity=1)

theta = 270 # we can attach encoder data to here
thetaChange = 0
while (True):
    rate(50)
    fore_arm.axis = vector(0, L*cos(theta*toRad), L*sin(theta*toRad))
    theta = theta + thetaChange
    if theta >=360:
        thetaChange = -.5
    if theta <= 270:
        thetaChange = +.5


