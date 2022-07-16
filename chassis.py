from math import atan2, sqrt, pi,sin,cos
import mainGraphics as mg
import time

x,y,currRotation = 0,0,0

class coordinate:
    def __init__(self, x, y):
        self.getX = x
        self.getY = y 

    
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

def distToPoint(x,y,px, py):
  return sqrt( (px-x)**2 + (py-y)**2 ) 

def sign(input):
    return(-1 if input < 0 else 1) 

def drive(d,timeout):
    global x,y,currRotation
    kp = 0.04
    dist = -d
    # dist = d

    tx = sin(2*pi-currRotation) * dist + x
    ty = cos(2*pi-currRotation) * dist + y

    for i in range(timeout):
        error = float(dist) - (dist-(distToPoint(x,y,tx,ty)))
        vel = kp*error * sign(d)
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
   try: 
    t = atan2(px-x,py-y)
   except:
    t = pi/2
   return t * (180/pi)  

#purePursuit/movement code

def moveToVel(target):
    global currRotation,x,y
    lkp = 0.1
    rkp = 0.08

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

    lVel = linearVel - rotationVel 
    rVel = linearVel + rotationVel 
    # print(lVel,rVel)
    return (lVel,rVel) 

def moveTo(target,timeout):
    speedLimit = 5
    for i in range(timeout):
        lVel,rVel = moveToVel(target) 
        lVel = lVel if lVel < speedLimit else speedLimit
        rVel = rVel if rVel < speedLimit else speedLimit
        odomStep(lVel,rVel)
        print(lVel,rVel)
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
    
    # robotPath.extend(((x,y),(px,py)))
    # robotPath.append((x,y))
    # robotPath.append((px,py))

    # if (pointInCircle(path[lineIndex + 1], lookAhead)):
    #     lineIndex += 1 

    while(True):
        target = targetPoint(path,lookAhead, lineLookAhead, lineIndex)

        if target == 1:
            lineIndex += 1
            print(lineIndex)
            target = targetPoint(path,lookAhead, lineLookAhead, lineIndex)
        
        if lineIndex == len(path)-2:
            break

        
        lVel,rVel = moveToVel(target) 
        odomStep(lVel,rVel) 
        # mg.drawLines(path)
        # mg.drawPoint(target.getX,target.getY)
        time.sleep(0.01)
        
    moveTo(path[-1],finalTimeout)
    
    

# path = [(288, 792),(457, 599),(498, 380),(283, 330),(506, 188),(331, 136)]
# p1 = [coordinate(288,792), coordinate(457,599), coordinate(498,380), coordinate(283,330), coordinate(506,188), coordinate(331,136)]

# x,px = p1[0].getX, p1[0].getX 
# y,py = p1[0].getY,p1[0].getY

 


