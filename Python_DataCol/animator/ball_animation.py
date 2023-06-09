from vpython import *
left = box(pos=vector(-5,0,0),length=.1,width=10,height=10,color=color.white,opacity=.2)
right = box(pos=vector(5,0,0),length=.1,width=10,height=10,color=color.white,opacity=.2)
top = box(pos=vector(0,5,0),length=10,width=10,height=.1,color=color.white,opacity=.2)
bottom = box(pos=vector(0,-5,0),length=10,width=10,height=.1,color=color.white,opacity=.2)
back = box(pos=vector(0,0,5),length=10,width=.1,height=10,color=color.white,opacity=.2)
front = box(pos=vector(0,0,-5),length=10,width=.1,height=10,color=color.white,opacity=.2)

ball=sphere(radius=.5, color=color.red)

xpos = 3
ypos = 0
zpos = -4

xchange = .1
ychange = .1
zchange = .1

while True:
    rate(50) #올리면 빠르게 공이 움직임
    ball.pos = vector (xpos, ypos, zpos)
    xpos = xpos + xchange
    ypos = ypos + ychange
    zpos = zpos + zchange

    if xpos>=5:
        xchange=-.1
    if xpos<=-5:
        xchange=.1
    if ypos>=5:
        ychange=-.1
    if ypos<=-5:
        ychange=.1
    if zpos>=5:
        zchange=-.1
    if zpos<=-5:
        zchange=.1