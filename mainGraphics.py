from math import cos, sin,dist
from tkinter import *
import time
import tkinter
import customtkinter as ct
# from chassis import x,y

robotFront = [0,0]

simulate = True

# ct.set_appearance_mode("System")
ct.set_default_color_theme("blue") 

def drawPoint(x,y):
    pythonGreen = "#476042"
    x1, y1 = (x - 5), (y - 5)
    x2, y2 = (x + 5), (y + 5)
    return(w.create_oval(x1, y1, x2, y2, width = 0, fill=pythonGreen))

def drawLine(p1,p2,color,width = 1):
    return(w.create_line(p1[0], p1[1], p2[0], p2[1],fill = color, width = width))


identifiers = []
a=[0,0]

def drawRobot(x,y,heading): 
    global identifiers,a,robotFront
    scaled = []
    points = [(x+49.5,y+49.5), (x-49.5,y+49.5), (x-49.5,y-49.5), (x+49.5,y-49.5)]
    # print(points)
    for i in range(4):
        px = points[i][0] - x
        py = points[i][1] - y
        scaled.append((px * cos(heading) - py * sin(heading) + x, py * cos(heading) + px * sin(heading) + y ))
    # w.create_oval(x-90,y-90,x+90,y+90)
    
    for i in range(2):
        w.delete(a[i])

    a[0] = (w.create_polygon(scaled[0][0], scaled[0][1], scaled[1][0],scaled[1][1],scaled[2][0],scaled[2][1],scaled[3][0],scaled[3][1],fill = "#F45B69"))
    a[1] = (drawLine(scaled[3], scaled[3-1],"#48BEFF", 5))
    robotFront[0] = scaled[3]
    robotFront[1] = scaled[3-1]
    # for i in range(3,-1,-1): 
    #   if not len(identifiers) == 4:
    #     if i == 3:
    #         identifiers.append(drawLine(scaled[i], scaled[i-1],"red"))
    #     else:
    #         identifiers.append(drawLine(scaled[i],scaled[i-1], "white"))
    #   else:
    #     # print(identifiers)
    #     w.delete(identifiers[i])
    #     identifiers.pop(i)
    #     if i == 3:
    #         identifiers.append(drawLine(scaled[i], scaled[i-1],"red"))
    #     else:
    #         identifiers.append(drawLine(scaled[i],scaled[i-1], "white"))
    

# def deleteRobot():
#     global identifiers
#     for i in range(4):
#         w.delete(identifiers[i])

def drawLines(path):
  for i in range(len(path)):
    # drawPoint(path[i][0], path[i][1])
    try:
        drawLine((path[i].getX, path[i].getY), (path[i+1].getX, path[i+1].getY),"black")
    except IndexError:
        pass


discsIdentifiers = []
discs = []

def drawDisc(x,y):
    global discs
    x1, y1 = (x - 15.1525), (y - 15.1525)
    x2, y2 = (x + 15.1525), (y + 15.1525)
    discsIdentifiers.append(w.create_oval(x1, y1, x2, y2, width = 0, fill="#FFFC99"))
    discs.append((x,y))

def drawField():
    w.create_line(360.4, 653.5, 360.4, 782.2, width = 5, fill = "#B2DBBF")
    w.create_line(100, 521.8, 228.7, 521.8, width = 5, fill = "#B2DBBF")
    w.create_line(357.4, 653.5, 357.4, 524.8, width = 11, fill = "#F45B69")
    w.create_line(228.7, 524.8, 362.5, 524.8, width = 11, fill = "#F45B69")
    w.create_line(611.8, 10, 611.8, 138.7, width = 5, fill = "#B2DBBF")
    w.create_line(614.4, 138.7, 614.4, 267.4, width = 11, fill = "#48BEFF")
    w.create_line(872.2, 270.4, 743.5, 270.4, width = 5, fill = "#B2DBBF")
    w.create_line(743.5, 267.4, 608.6, 267.4, width = 11, fill = "#48BEFF")
    w.create_oval(150.5925, 640.9425, 241.1075, 731.4575, width = 0, fill = "#48BEFF")
    w.create_oval(732.4175, 63.2675, 818.9325, 149.7825, width = 0, fill = "#F45B69")
    w.create_line(614.8, 782.2, 614.8, 717.85, width = 5, fill = "#B2DBBF")
    w.create_line(872.2, 653.5, 807.85, 653.5, width = 5, fill = "#B2DBBF")
    w.create_line(100, 138.7, 164.35, 138.7, width = 5, fill = "#B2DBBF")
    w.create_line(357.4, 10, 357.4, 74.35, width = 5, fill = "#B2DBBF")
    w.create_line(107, 5, 877.2, 775.2, width = 5, fill = "#B2DBBF")
    w.create_line(95, 17, 865.2, 787.2, width = 5, fill = "#B2DBBF")

def getDiscs():
    return discsIdentifiers


def populateField():
    sx = 100
    sy = 10
    bx = 872.2
    by = 782.2
    ts = 128.7
    for i in range(5):

        drawDisc(sx + ts/2 + ts/2 * i, sy + ts/2 + ts/2*i)
        drawDisc(bx - ts/2 - ts/2 * i, by - ts/2 - ts/2 * i)

    for i in range(3):
        offset = 98.395/2

        drawDisc(sx + 2*ts  - 15.1525 - offset*i, by - 2*ts -15.1525 - 7)
        drawDisc(sx + 2*ts + 15.1525 + 7 , by - 2*ts + 15.1525 + offset*i)
        drawDisc(sx + 5 * ts - 15.1525 - offset*i, sy + 2*ts + 15.1525+7)
        drawDisc(sx + 4*ts - 15.1525 - 7, sy + 2*ts - 15.1525 - offset*i)
        drawDisc(sx + 3 * ts + ts/2*i, sy + 4 * ts + ts/2*i)
        drawDisc(sx + 4 * ts - ts/2*i - ts/2 , sy + ts * 2 - ts/2*i + ts/2)

    drawDisc(sx + 2 * ts - ts/2,sy + 2 * ts + ts/2)
    drawDisc(sx + 4 * ts + ts/2,sy + 3 * ts + ts/2)

discsVisible = True

def toggleDiscs(event):
    global discsVisible
    if(discsVisible):
        for i in range(len(getDiscs())):
            w.delete(getDiscs()[i])
        discsVisible = False
    else:
        populateField()
        discsVisible = True

def checkContact():
    global discs,robotFront
    while True:
        points = robotFront
        x1,x2,y1,y2 = points[0][0], points[1][0], points[0][1], points[1][1]

        midpoint = ((x1+x2)/2,(y1+y2)/2)

        for i in range(len(discs)):
            dx = discs[i][0]
            dy = discs[i][1]
            distance = dist(midpoint,(dx,dy))
            if (distance < 20):
                removeDisc(i)
        time.sleep(0.01)

def removeDisc(index):
    w.delete(discsIdentifiers[index])

def tile(x,y):
    sx = 100
    sy = 10
    ts = 128.7
    return(sx + ts * x - ts/2,sy + y *ts -ts/2)
# def drawDisc(event):
#     x1, y1 = (event.x - 15.1525), (event.y - 15.1525)
#     x2, y2 = (event.x + 15.1525), (event.y + 15.1525)
#     discs.append((event.x,event.y))
#     print(discs)
#     w.create_oval(x1, y1, x2, y2, width = 0, fill="#FFFC99")


# path = [(288, 792),(457, 599),(498, 380),(283, 330),(506, 188),(331, 136)]
# p1 = [coordinate(288,792), coordinate(457,599), coordinate(498,380), coordinate(283,330), coordinate(506,188), coordinate(331,136)]

# x,px = p1[0].getX, p1[0].getX 
# y,py = p1[0].getY,p1[0].getY

#graphics 

def toggleEdit():
    global simulate
    simulate = False
    button.place_forget()

canvas_width = 18*40 + 270
canvas_height = 18*40 + 100

master = Tk()
master.title("g")
w = Canvas(master, width=canvas_width, height=canvas_height,background = "#B2DBBF")
editIcon = tkinter.PhotoImage(file = "robotics/vexRobotSimulator/assets/edit button.png")
button = ct.CTkButton(master=w, text = None, image = editIcon.subsample(4), fg_color = "#B2DBBF", width = 40,height = 40,command = toggleEdit)
button.place(x=25, y=410)

#772.2 x 772.2

# (100,802)
# (892,10)
# (100,10)
# (892,802)

# (100,782.2)
# (872.2,10)
# (100,10)
# (872.2,782.2)

#i for x
#j for y

colors = ["#6699CC", "#70A6DC"]
for i in range(6):
    for j in range(6):
        cx = 100 + i * 128.7
        cy = 782.2 - j * 128.7
        w.create_rectangle(cx,cy,cx+128.7,cy-128.7,fill = colors[j%2 - i%2],outline= "#F45B69")

# w.create_rectangle(100 ,782.2 ,892 ,10,fill = "#305252") 

for i in range(1,6):
    w.create_line(100,782.2 - 128.7*i, 100,782.2 - 128.7*i, fill = "#F45B69")
    w.create_line(100 + 128.7*i,782.2, 100+128.7*i,10, fill = "#F45B69")


#12*12*5.5
w.pack(expand=YES, fill=BOTH)


# drawRobot(path[0][0], path[0][1],0)


# mainloop()