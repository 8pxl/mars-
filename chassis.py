from math import atan2, sqrt, pi,sin,cos, asin
from numpy import arange

import sympy
import mainGraphics as mg
import time
from sympy import diff,symbols 

x,y,px,py,currRotation = 0,0,0,0,0

def sign(input):
    return(-1 if input < 0 else 1) 

def distToPoint(x,y,px, py):
  return sqrt( (px-x)**2 + (py-y)**2 ) 

class coordinate:
    def __init__(self, x, y):
        self.getX = x
        self.getY = y 


class bezier:

    def __init__(self,p0,p3,initialWeight,finalWeight,initialHeading,finalHeading):
        self.p0 = p0
        self.p1 = (p0[0] + sin(initialHeading) * initialWeight, p0[1] + cos(initialHeading) * initialWeight)
        self.p2 = (p3[0] + sin(pi/2 + (pi/2-finalHeading)) * -1 * finalWeight, p3[1] + cos(pi/2 + (pi/2-finalHeading)) * -1*finalWeight)
        self.p3 = p3
        mg.drawPoint(p0[0],p0[1])
        mg.drawPoint(self.p1[0],self.p1[1],"#FFFFFF")
        mg.drawPoint(self.p2[0],self.p2[1],"#FFFFFF")
        mg.drawPoint(p3[0],p3[1])
    
    def solve(self,t):
        omt = 1-t
        x0 = self.p0[0]
        x1 = self.p1[0]
        x2 = self.p2[0]
        x3 = self.p3[0]
        y0 = self.p0[1]
        y1 = self.p1[1]
        y2 = self.p2[1]
        y3 = self.p3[1]
        return( pow(omt,3) * x0 + 3 * pow(omt,2) * t * x1 + 3*omt * pow(t,2) * x2 + pow(t,3) * x3, pow(omt,3) * y0 + 3 * pow(omt,2) * t * y1 + 3*omt * pow(t,2) * y2 + pow(t,3) * y3)

    def tangentLineAngle(self,t):
        x0 = self.p0[0]
        x1 = self.p1[0]
        x2 = self.p2[0]
        x3 = self.p3[0]
        y0 = self.p0[1]
        y1 = self.p1[1]
        y2 = self.p2[1]
        y3 = self.p3[1]

        ex = symbols('x')
        omt = 1-ex
        # bx = pow(omt,3) * x0 + 3 * pow(omt,2) * t * x1 + 3*omt * pow(t,2) * x2 + pow(t,3) * x3
        # by = pow(omt,3) * y0 + 3 * pow(omt,2) * t * y1 + 3*omt * pow(t,2) * y2 + pow(t,3) * y3

        bx = pow(omt,3) * x0 + 3 * pow(omt,2) * ex * x1 + 3*omt * pow(ex,2) * x2 + pow(ex,3) * x3
        by = pow(omt,3) * y0 + 3 * pow(omt,2) * ex * y1 + 3*omt * pow(ex,2) * y2 + pow(ex,3) * y3
        

        bpx = diff(bx,ex).evalf(subs={ex: t})
        bpy = diff(by,ex).evalf(subs={ex: t})

        m = bpy/bpx

        return(atan2(1,m))
    
    def createLUT(self, resolution):
        points = []
        angles = []
        for i in range(1,resolution+1):
            points.append(self.solve(i/resolution))
            angles.append(self.tangentLineAngle(i/resolution))
        return [points, angles]

    def approximateLength(self, lut, resolution):
        length = 0
        for i in range(resolution - 1):
            p0 = lut[0][i]
            p1 = lut[0][i+1]
            length += distToPoint(p0[0],p0[1],p1[0],p1[1])
        return length


    
def odomStep(dl,dr):
    global x,y,currRotation
    #currHeading -> -180-180
    offset = 25
    deltaRotation = (dl-dr) / 50
    
    currRotation += deltaRotation
    if deltaRotation == 0:
        deltaY = cos(2*pi-currRotation) * dr
        deltaX = sin(2*pi-currRotation) * dr

    else:
        r = dr/deltaRotation + offset

        relativeY = 2*sin(deltaRotation/2) * r 

        rotationOffset = currRotation+deltaRotation/2
        theta = pi/2
        radius = relativeY
        theta += rotationOffset
        deltaX = radius*cos(theta)
        deltaY = radius*sin(theta)
    
    x -= deltaX
    y -= deltaY
    mg.drawRobot(x,y,currRotation)

def drive(d,timeout):
    global x,y,currRotation
    speedLimit = 5
    kp = 0.04
    dist = -d
    # dist = d

    tx = sin(2*pi-currRotation) * dist + x
    ty = cos(2*pi-currRotation) * dist + y

    for i in range(timeout):
        error = float(dist) - (dist-(distToPoint(x,y,tx,ty)))
        vel = kp*error * sign(d)
        vel = vel if vel < speedLimit else speedLimit
        odomStep(vel,vel)
        time.sleep(0.01)

def rotate(target,timeout):
    global x,y,currRotation
    kp = 0.04

    for i in range(timeout):
        scaled = rtd(currRotation) if currRotation > 0 else rtd(currRotation) + 360
        error = minError(target,(scaled))
        dir = -dirToSpin(target,(scaled))
        vel = error*kp*dir
        odomStep(vel,-vel)
        time.sleep(0.01)

def minError(target,current):
    b = max(target,current)
    s = min(target,current)
    diff = b - s
    if diff <= 180:
        return(diff)
    else:
        return(360-b + s)

def dtr(input):
  return(pi*input/180)

def rtd( input):
  return(input*180/pi)


def dirToSpin(target,currHeading):
  # -1 = clockwise
  # 1 = counterclockwise
  # currHeading = unscaled if unscaled < 180 else unscaled - 360
  diff = target - currHeading;
  if(diff < 0):
      diff += 360
  if(diff > 180):
      return 1
  else:
      return -1

def absoluteAngleToPoint(px,py):
   global x,y
#    print(x,y)
   try: 
    t = atan2(px-x,py-y)
   except:
    t = pi/2
   return t * (180/pi)  

#purePursuit/movement code

def moveToVel(target):
    global currRotation,x,y
    lkp = 0.1
    rkp = 0.1
    krp = 1.2
    tx = target.getX
    ty = target.getY

    linearError = distToPoint(x,y,tx, ty)
    linearVel = linearError*lkp 

    currHeading = rtd(currRotation) if currRotation > 0 else rtd(currRotation) + 360 # 0-360

    targetHeading = 180-absoluteAngleToPoint(tx, ty) # -180-180

    # drawRobot(100,500,dtr(targetHeading))

    targetHeading = targetHeading if targetHeading >= 0 else abs(targetHeading) + 180

    dir = dirToSpin(targetHeading,currHeading)  
    
    # print(targetHeading,currHeading)
    # print(dir)

    rotationError = minError(targetHeading,currHeading)
    # print(diff)

    rotationVel = rotationError * rkp * dir 

    lVel = (linearVel - abs(rotationVel) * krp) - rotationVel 
    rVel = (linearVel - abs(rotationVel) * krp) + rotationVel 
    return (lVel,rVel) 

def moveTo(target,timeout):
    speedLimit = 4
    for i in range(timeout):
        lVel,rVel = moveToVel(target) 
        print(lVel,rVel)
        lVel = lVel if lVel < speedLimit else speedLimit
        rVel = rVel if rVel < speedLimit else speedLimit
        odomStep(lVel,rVel)
        # print(lVel,rVel)
        time.sleep(0.01)

def targetPoint(path, lookAhead, lineLookAhead, lineIndex):
    global x
    global y

    # farthestPoint = coordinate( path[lineLookAhead - 1].getX, path[lineLookAhead - 1].getY)
    targetPoint   = coordinate(0,0) 
    closestDist = 1000000000 

    a = lineIndex+lineLookAhead if lineLookAhead < (len(path) - lineIndex) else len(path) -1 
    for i in range(lineIndex, a):

        farthestPoint = coordinate( path[a-1].getX, path[a - 1].getY)
        # drawLine((x,y),(farthestPoint.getX, farthestPoint.getY))
        # print(range(lineIndex, lineIndex+lineLookAhead if lineIndex+lineLookAhead < len(path) - lineIndex - 1 else len(path) - lineIndex - 1))

        x1 = path[i].getX
        y1 = path[i].getY
        x2 = path[i+1].getX
        y2 = path[i+1].getY

        ox1 = x1 - x 
        oy1 = y1 - y 
        ox2 = x2 - x 
        oy2 = y2 - y 

        dx = ox2-ox1 
        dy = oy2-oy1 
        dr = sqrt(pow(dx,2)+pow(dy,2)) 
        D = ox1*oy2 - ox2 * oy1 

        discriminant = pow(lookAhead,2)  *  pow(dr,2) - pow(D,2) 
        if (discriminant >= 0):
              sDiscriminant = sqrt(discriminant) 
              dxdy = D * dy 
              dxdx = D*dx 
              sdyxdxxsd = sign(dy) * dx * sDiscriminant 
              dr2 = pow(dr,2) 
              adyxsd = abs(dy) * sDiscriminant 

              minX = min(x1,x2) 
              maxX = max(x1,x2) 
              minY = min(y1,y2) 
              maxY = max(y1,y2) 

              sx1 = (dxdy + sdyxdxxsd) / dr2 
              sy1 = (-dxdx + adyxsd) / dr2 
              sx2 = (dxdy - sdyxdxxsd) / dr2 
              sy2 = (-dxdx - adyxsd) / dr2 

              s1= [sx1+x,sy1+y]
              s2 = [sx2+x,sy2+y]

              # drawPoint(s1[0],s1[1])
              # drawPoint(s2[0],s2[1])

              s1Valid = s1[0] >= minX and s1[0] <= maxX and s1[1] >= minY and s1[1] <= maxY 
              s2Valid = s2[0] >= minX and s2[0] <= maxX and s2[1] >= minY and s2[1] <= maxY 

              #if point is found in the index of the next line segment, exit and update lineIndex  
              if i == a-1 and not (a-1 == len(path)):
                if(s1Valid):
                  return(1)
                if(s2Valid):
                  return(1)

              s1Dist = distToPoint(s1[0],s1[1],farthestPoint.getX,farthestPoint.getY) 
              s2Dist = distToPoint(s2[0],s2[1],farthestPoint.getX, farthestPoint.getY) 

              if (s1Valid and s1Dist < closestDist):
                targetPoint = coordinate(s1[0],s1[1]) 
                closestDist = s1Dist 

              if (s2Valid and s2Dist < closestDist):
                targetPoint  = coordinate(s2[0],s2[1]) 
                closestDist = s2Dist 
    
    return(targetPoint)



robotPath = []

def moveToPurePursuit(path, lookAhead, lineLookAhead,finalTimeout):
    global px,py,x,y
    lineIndex = 0
    speedLimit = 4
    mg.drawLines(path)
    # if (pointInCircle(path[lineIndex + 1], lookAhead)):
    #     lineIndex += 1 

    while(True):
        target = targetPoint(path,lookAhead, lineLookAhead, lineIndex)

        if target == 1:
            lineIndex += 1
            print(lineIndex)
            target = targetPoint(path,lookAhead, lineLookAhead, lineIndex)
        
        # if lineIndex == len(path)-2:
        #     break

        
        lVel,rVel = moveToVel(target) 
        lVel = lVel if lVel < speedLimit else speedLimit
        rVel = rVel if rVel < speedLimit else speedLimit
        odomStep(lVel,rVel) 

        # robotPath.append(coordinate(x,y))
        # robotPath.append(coordinate(px,py))

        # px,py = x,y
        # mg.drawPoint(target.getX,target.getY)
        time.sleep(0.01)

    # mg.drawLines(robotPath)
    moveTo(path[-1],finalTimeout)


def moveToPosePID(tx,ty,finalHeading,initialBias,finalBias, timeout, initialHeading = currRotation):
    global x,y,currRotation
    offset = 25

    curve = bezier((x,y),(tx,ty),initialBias,finalBias,currRotation,dtr(finalHeading))

    for i in range(100):
        mg.drawPoint(curve.solve(i/100)[0],curve.solve(i/100)[1])
    step = 1/timeout
    vel = 0.1
    aVel = 0.1

    for i in range(timeout):
        d = step*(i+1)
        # targetHeading = curve.tangentLineAngle(d)
        targetPos = curve.solve(d)
        # print(targetPos)
        targetPos = coordinate(targetPos[0],targetPos[1])
        lVel,rVel = moveToVel(targetPos)
        odomStep(lVel,rVel)
        time.sleep(0.01)

    moveTo(targetPos,1000)
    
def linearVelocityProfileFromDist(dist,maxAccel,maxSpeed):

    #in milliseconds
    profile = []
    vel = 0

    # dist = distToPoint(path[0][0],path[0][1],path[1][0],path[1][1])

    accelTime = int(maxSpeed / maxAccel)
    accelDist = maxAccel * accelTime                
    cruiseDist = dist - (2 * accelDist)
    cruiseTime = int(cruiseDist / maxSpeed)

    for i in range(0,accelTime):
        profile.append((vel,vel))
        vel += maxAccel
    for j in range(0,cruiseTime):
        profile.append((vel,vel))
    for k in range(0,accelTime):
        profile.append((vel,vel))
        vel -= maxAccel
    return(profile)


def createMotionProfileFromCurve(curve, maxAccel, maxSpeed, maxAngular,resolution,rotationBias):

    for i in range(100):
        mg.drawPoint(curve.solve(i/100)[0],curve.solve(i/100)[1])

    lkp = 0.1
    rkp = 0.1
    motionProfile = []
    
    print(resolution)
    lut = curve.createLUT(resolution)

    dist = curve.approximateLength(lut, resolution)

    linearVelocityProfile = linearVelocityProfileFromDist(dist,maxAccel,maxSpeed)
    #in milliseconds
    totalTime = len(linearVelocityProfile)
    timeIncrement = resolution/totalTime
    for i in range(resolution-1):
        # mg.drawRobot(x,y,pi-lut[1][i])
        # time.sleep(0.01)  
        targetPoint = lut[0][i+1]
        targetHeading = lut[1][i+1]
        vel = maxSpeed - (abs(lut[1][i+1] - lut[1][i])) * rotationBias


        motionProfile.append()
        # print(maxAllowableVelocity)   
    return(motionProfile)

def velocitesToPose(target,heading):
    global x,y,currRotation
    #currHeading -> -180-180
    offset = 25

    deltaRotation = heading-currRotation
    # deltaRotation = (dl-dr) / 50
    
    if deltaRotation == 0:
        deltaY = cos(2*pi-currRotation) * dr
        deltaX = sin(2*pi-currRotation) * dr

    else:
        r = dr/deltaRotation + offset

        relativeY = 2*sin(deltaRotation/2) * r 

        rotationOffset = currRotation+deltaRotation/2
        theta = pi/2
        radius = relativeY
        theta += rotationOffset
        deltaX = radius*cos(theta)
        deltaY = radius*sin(theta)
    
    x -= deltaX
    y -= deltaY

def followMotionProfile(profile):
    for i in range(len(profile)):
        odomStep(profile[i][0],profile[i][1])
        time.sleep(0.001)

# path = [(288, 792),(457, 599),(498, 380),(283, 330),(506, 188),(331, 136)]
# p1 = [coordinate(288,792), coordinate(457,599), coordinate(498,380), coordinate(283,330), coordinate(506,188), coordinate(331,136)]

# x,px = p1[0].getX, p1[0].getX 
# y,py = p1[0].getY,p1[0].getY

 


