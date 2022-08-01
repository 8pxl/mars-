from math import pi
import threading
from tkinter import *
import mainGraphics as mg
import chassis as c
import pathGeneration as pg
import flywheel as f
import time


c.x,c.y = mg.tile(1,3)
# c.y += 7
c.currRotation = pi/2

# x1,y1,x2,y2 = [732.4175, 63.2675, 818.9325, 149.7825]

ts = 128.7
sx = 100
sy = 10

goal = (sx+5*ts + 1/4*ts,sy+ 3/4 * ts)
goal2 = (sx + 3/4*ts, sy + 5*ts + 1/4*ts)

#all timeouts in centiseconds


# one =[c.coordinate(400,201), c.coordinate(357,261), c.coordinate(290,195), c.coordinate(229,136)]

one = [c.coordinate(c.x,c.y), c.coordinate(243,456), c.coordinate(286,456), c.coordinate(335,457), c.coordinate(407,455), c.coordinate(434,424)]

two = [c.coordinate(413,463), c.coordinate(487,518), c.coordinate(615,659),c.coordinate(615,659)]

three = [c.coordinate(577,512), c.coordinate(679,584), c.coordinate(740,644), c.coordinate(811,716)]

four = [c.coordinate(797,700), c.coordinate(622,654), c.coordinate(553,587), c.coordinate(483,522), c.coordinate(445,458), c.coordinate(526,406)]

five = [c.coordinate(527,395), c.coordinate(678,457), c.coordinate(692,394), c.coordinate(698,369)]

heart = [c.coordinate(160,318), c.coordinate(295,184), c.coordinate(452,297), c.coordinate(573,192), c.coordinate(714,316), c.coordinate(741,459), c.coordinate(588,595), c.coordinate(480,689), c.coordinate(325,592), c.coordinate(200,466), c.coordinate(156,327)]

def chassisLoop():

    curve = c.bezier((c.x,c.y), (mg.tile(3,4)[0],mg.tile(3,4)[1]),45,45,c.currRotation,c.dtr(90))
    c.createMotionProfileFromCurve(curve,0.1,0.5,0.5,100)

    # ix = c.x
    # profile = c.createMotionProfileFromLine(((c.x,c.y), (c.x + 200,c.y)),0.1,0.5)
    # print(profile)
    # c.followMotionProfile(profile)
    # time.sleep(0.8)
    # mg.drawRobot(ix + 200,c.y,c.currRotation)
    # c.moveToPosePID(mg.tile(3,4)[0],mg.tile(3,4)[1],45,300,50,100)
    # c.moveTo(c.coordinate(413,463),100)
    # c.rotate(90,100)
    # c.moveToPurePursuit(heart,20,2,50)    
    

        
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
        mg.toggleDiscs()
    else:
        pg.showPoints(event)


mg.drawRobot(c.x,c.y,c.currRotation)

mg.w.bind("<Button-1>", something)
mg.w.bind("<Button-2>", something2)

mg.drawField()
mg.populateField()
mg.mainloop()