import threading
from tkinter import *
import mainGraphics as mg
import chassis as c
import time


x,y,currHeading = 300,730,0

discsVisible = True

#all timeouts in centiseconds

path = [(300, 630),(457, 599),(498, 380)]
p1 = [c.coordinate(300,630), c.coordinate(457,599), c.coordinate(498,380)]

def chassisLoop():
    c.drive(100,100)
    c.rotate(90,100)
    c.moveToPurePursuit(p1,90,2,100)
    c.rotate(0,100)
    c.drive(-300,150)
    # time.sleep(1)
        
def something(event):
    robot = threading.Thread(target = chassisLoop)
    robot.start()

def toggleDiscs(event):
    global discsVisible
    if(discsVisible):
        for i in range(len(mg.getDiscs())):
            mg.w.delete(mg.getDiscs()[i])
        discsVisible = False
    else:
        mg.populateField()
        discsVisible = True


mg.drawRobot(x,y,currHeading)
mg.w.bind("<Button-1>", something)
mg.w.bind("<Button-2>", toggleDiscs)

mg.populateField()
mg.mainloop()