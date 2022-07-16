from math import pi
import threading
from tkinter import *
import mainGraphics as mg
import chassis as c
import pathGeneration as pg


c.x,c.y = mg.tile(1,3)
c.currRotation = pi/2

x1,y1,x2,y2 = [732.4175, 63.2675, 818.9325, 149.7825]
goal = ((x1+x2)/2,(y1+y2)/2)

#all timeouts in centiseconds

path = [(300, 630),(457, 599),(498, 380)]
first = [c.coordinate(257,335), c.coordinate(429,455), c.coordinate(485,519), c.coordinate(614,644), c.coordinate(673,717)]

def chassisLoop():
    c.drive(100,50)
    c.rotate(c.absoluteAngleToPoint(goal[0],goal[1]) + 90,100)
    c.moveToPurePursuit(first,20,2,200)
    # c.rotate(90,80)
    # c.drive(300,50)

        
def something(event):
    if(mg.simulate):
        robot = threading.Thread(target = chassisLoop)
        contact = threading.Thread(target = mg.checkContact)
        robot.start()
        contact.start()
    else:
        pg.exportButton.place(x = 10,y = 10)
        pg.createPath(event)

def something2(event):
    if(mg.simulate):
        mg.toggleDiscs
    else:
        pg.showPoints(event)


mg.drawRobot(c.x,c.y,c.currRotation)

mg.w.bind("<Button-1>", something)
mg.w.bind("<Button-2>", something2)

mg.drawField()
mg.populateField()
mg.mainloop()