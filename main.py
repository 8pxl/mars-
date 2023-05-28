from math import pi
import threading
from tkinter import *
import mainGraphics as mg
import chassis as c
import pathGeneration as pg
import flywheel as f
import time
import physics as ph
# import driver as d



c.x,c.y = mg.tile(3,3)
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

infinite = [c.coordinate(165,329), c.coordinate(234,251), c.coordinate(295,193), c.coordinate(352,249), c.coordinate(407,339), c.coordinate(475,395), c.coordinate(548,353), c.coordinate(578,288), c.coordinate(586,221), c.coordinate(563,179), c.coordinate(514,189), c.coordinate(465,222), c.coordinate(425,278), c.coordinate(344,335), c.coordinate(251,384), c.coordinate(175,361), c.coordinate(165,331)]

p = [c.coordinate(159,341), c.coordinate(282,213), c.coordinate(459,226), c.coordinate(553,349), c.coordinate(494,473), c.coordinate(316,422), c.coordinate(240,574), c.coordinate(216,434), c.coordinate(160,336)]
def chassisLoop():  

    # a = c.linearVelocityProfileFromDist(70,0.1,3)
    # c.followMotionProfile(a)
    # for i in range(50):
    #     c.odomStep(9,4)
    #     time.sleep(0.01)
    c.rotate(180-c.absoluteAngleToPoint(goal2[0],goal2[1]),50)
    mg.numDiscs = 9
    f.shoot(9,goal2,10,0.14);
    curve = c.bezier((c.x,c.y), (mg.tile(2,3)[0],mg.tile(2,3)[1]),150,120, pi-c.currRotation, c.dtr(195.25511870305778))

    c.fakeFollowCurve(curve,100)
    # c.rotate(90,80)
    # c.drive(60,80)
    c.rotate(180-c.absoluteAngleToPoint(goal2[0],goal2[1]),50)
    f.shoot(3,goal2,10,0.14)
    curve = c.bezier((c.x,c.y), (229, 470) ,150,200, pi-c.currRotation, pi/2)
    c.fakeFollowCurve(curve,100)
    c.drive(60,80)
    c.rotate(180-c.absoluteAngleToPoint(goal2[0],goal2[1]),50)
    c.drive(-60,80)
    f.shoot(3,goal2,10,0.14)
    c.rotate(0,90)
    c.drive(400,90)
    c.drive(-60,70)
    c.rotate(-90,80)
    c.drive(150,70)
    c.drive(-60,70)
    c.rotate(90,80)
    c.drive(250,100)
    c.rotate(180-c.absoluteAngleToPoint(goal[0],goal[1]),50)
    f.shoot(3,goal,10,0.14)
    curve = c.bezier((c.x,c.y), (560, 151) ,100,200, pi-c.currRotation,pi)
    c.fakeFollowCurve(curve,100)
    c.drive(200,100)
    c.rotate(180-c.absoluteAngleToPoint(goal[0],goal[1]),50)
    f.shoot(3,goal,10,0.14)
    c.drive(40,40)
    c.drive(-40,40)
    c.rotate(315,70)
    curve = c.bezier((c.x,c.y), (560, 151) ,500,100, pi-c.currRotation,pi/2)
    c.fakeFollowCurve(curve,230)
    c.rotate(180-c.absoluteAngleToPoint(goal[0],goal[1]),50)
    f.shoot(3,goal,10,0.14)
    c.rotate(180,60)
    curve = c.bezier((c.x,c.y), (681, 300),150,150, pi-c.currRotation,pi/2)
    c.fakeFollowCurve(curve,100)
    c.drive(100,80)
    c.rotate(180-c.absoluteAngleToPoint(goal[0],goal[1]),50)
    f.shoot(3,goal,10,0.14)

    




    curve = c.bezier((c.x,c.y), (mg.tile(4,1)[0],mg.tile(4,1)[1]),45,150,c.currRotation,c.dtr(0))
    # c.fakeFollowCurve(curve,100)
    # for i in range(100):
    #     mg.drawPoint(curve.solve(i/100)[0],curve.solve(i/100)[1]) 

    # c.moveToPosePID(mg.tile(4,1)[0],mg.tile(4,1)[1], 0, 45, 200, 1000, c.currRotation)
    a = c.createMotionProfile(curve,0.4,3,0.1,100)
    # a = c.motionProfileTest(curve,100)
    c.followMotionProfile(a)    

    # point = (300,400)
    # dl,dr = c.distancesToPose((c.x,c.y),point,pi,pi/2)
    # mg.drawPoint(point[0],point[1]) 
    # time.sleep(1)
    # x,y = c.odomStep(dl,dr,True,(c.x,c.y),pi/2)

    # el,er = c.otherDistancesToPose((x,y),point,pi,pi)
    # c.odomStep(dl,dr)
    # c.odomStep(el,er)
    
    # c.moveToPurePursuit(p,80,2,50);
    # c.moveTo(c.coordinate(200,200),1000)
    # c.keejController(curve,10);
    # dl,dr = c.velocitesToPose((400,700), 90)
    # mg.drawLine((c.x,c.y),(400,700))
    # mg.drawPoint(700,600);    
    # c.moveTo(c.coordinate(700,600), 1000)
    # c.x = 400
    # c.y = 430
    # c.drive(100,100)
    # c.rotate(135,100)
    # c.rotate(180- c.absoluteAngleToPoint(goal[0], goal[1]), 100)
    # f.shoot(3,goal,100,0.1)
    # c.createMotionProfileFromCurve(curve,0.1,0.5,0.5,100)

    # ix = c.x
    # profile = c.createMotionProfileFromLine(((c.x,c.y), (c.x + 200,c.y)),0.1,0.5)
    # print(profile)
    # c.followMotionProfile(profile)
    # time.sleep(0.8)
    # mg.drawRobot(ix + 200,c.y,c.currRotation)
    # c.moveToPosePID(mg.tile(5,5)[0],mg.tile(5,5)[1],180,50,300,100)
    # c.moveTo(c.coordinate(413,463),100)
    # c.rotate(90,100)
    # c.moveToPurePursuit(p,20,2,50)    
    

        
def something(event):
    if(mg.simulate):
        robot = threading.Thread(target = chassisLoop)
        contact = threading.Thread(target = mg.checkContact)
        driver = threading.Thread(target = ph.moveRobot)
        # driver.start()
        contact.start()
        robot.start()
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