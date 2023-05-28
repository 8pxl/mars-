import time
from turtle import speed
import chassis as c
import driver as d

leftVelocity,rightVelocity = 0,0
accel = 0.8
decel = 0.1
speedLimit = 5
ts = 128.7
sx = 100
sy = 10

def moveRobot():
    global leftVelocity,rightVelocity,accel,speedLimit
    while True:
        
        # if d.keysPressing[0]:
        #     leftVelocity += accel
        #     rightVelocity += accel
        # if d.keysPressing[1]:
        #     leftVelocity += accel
        #     rightVelocity -= accel
        #     if d.keysPressing[0]:
        #         rightVelocity -= accel/2
        #     if d.keysPressing[2]:
        #         leftVelocity += accel/2

        # if d.keysPressing[2]:
        #     leftVelocity -= accel
        #     rightVelocity -= accel
        # if d.keysPressing[3]:
        #     leftVelocity -= accel
        #     rightVelocity += accel

        #     if d.keysPressing[0]:
        #         leftVelocity -= accel/2
        #     if d.keysPressing[2]:
        #         rightVelocity += accel/2

        if d.keysPressing[0]:
            c.y-=3
        if d.keysPressing[1]:
            c.x -= 3

        if d.keysPressing[2]:
            c.y +=3
        if d.keysPressing[3]:
            c.x+=3


        if d.keysPressing[4]:
            leftVelocity -= accel/4
            rightVelocity += accel/4

        if d.keysPressing[5]:
            leftVelocity += accel/4
            rightVelocity -= accel/4
                
        # robotAccelerating = d.keysPressing[0] or d.keysPressing[1] or d.keysPressing[2] or d.keysPressing[3]
        robotAccelerating = d.keysPressing[4] or d.keysPressing[5]
        # robotAccelerating = False


        if not robotAccelerating:
            if abs(rightVelocity) < decel:
                rightVelocity = 0
            else:
                rightVelocity = rightVelocity - decel if rightVelocity > 0 else rightVelocity + decel

            if abs(leftVelocity) < decel:
                leftVelocity = 0
            else:
                leftVelocity = leftVelocity - decel if leftVelocity > 0 else leftVelocity + decel

        if abs(leftVelocity) >= speedLimit:
            leftVelocity = speedLimit * c.sign(leftVelocity);

        if abs(rightVelocity) >= speedLimit:
            rightVelocity = speedLimit * c.sign(rightVelocity);

        c.odomStep(rightVelocity,leftVelocity)

        # print(-c.absoluteAngleToPoint(0,0),c.rtd(c.currRotation))

        # c.currRotation = c.dtr(-c.absoluteAngleToPoint(sx+5*ts + 1/4*ts,sy+ 3/4 * ts))
        c.currRotation = c.dtr(180-c.absoluteAngleToPoint(sx + 3/4*ts, sy + 5*ts + 1/4*ts))
        time.sleep(0.01)

