from math import pi
import threading
from tkinter import *
import mainGraphics as mg
import chassis as c
import pathGeneration as pg
import flywheel as f

c.x,c.y = mg.tile(1,3)
c.currRotation = pi/2

# x1,y1,x2,y2 = [732.4175, 63.2675, 818.9325, 149.7825]

ts = 128.7
sx = 100
sy = 10

goal = (sx+5*ts + 1/4*ts,sy+ 3/4 * ts)

#all timeouts in centiseconds


one =[c.coordinate(400,201), c.coordinate(357,261), c.coordinate(290,195), c.coordinate(229,136)]

two = [c.coordinate(235,134), c.coordinate(161,76), c.coordinate(232,40), c.coordinate(372,49), c.coordinate(540,452), c.coordinate(617,522)]

three = [c.coordinate(577,512), c.coordinate(679,584), c.coordinate(740,644), c.coordinate(811,716)]

four = [c.coordinate(797,700), c.coordinate(622,654), c.coordinate(553,587), c.coordinate(483,522), c.coordinate(445,458), c.coordinate(526,406)]

five = [c.coordinate(527,395), c.coordinate(678,457), c.coordinate(692,394), c.coordinate(698,369)]

def chassisLoop():
    c.drive(400,90)
    c.rotate(180-c.absoluteAngleToPoint(goal[0],goal[1]),70)
    f.shoot(3,goal,10,0.3)
    c.rotate(315,60)
    c.drive(200,80)
    c.rotate(180-c.absoluteAngleToPoint(goal[0],goal[1]),100)
    f.shoot(2,goal,10,0.3)
    c.moveToPurePursuit(one,30,2,40)
    c.rotate(180-c.absoluteAngleToPoint(goal[0],goal[1]),70)
    f.shoot(3,goal,10,0.3)
    c.moveToPurePursuit(two,30,2,40)
    c.rotate(180-c.absoluteAngleToPoint(goal[0],goal[1]),70)
    f.shoot(3,goal,10,0.3)
    c.moveToPurePursuit(three,30,2,40)
    c.rotate(180-c.absoluteAngleToPoint(goal[0],goal[1]),70)
    f.shoot(3,goal,10,0.3)
    c.moveToPurePursuit(four,30,2,40)
    c.rotate(180-c.absoluteAngleToPoint(goal[0],goal[1]),20)
    f.shoot(3,goal,10,0.3)
    

        
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