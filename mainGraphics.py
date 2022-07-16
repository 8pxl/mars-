from math import cos, sin
from tkinter import *

from matplotlib.pyplot import draw

def drawPoint(x,y):
    pythonGreen = "#476042"
    x1, y1 = (x - 5), (y - 5)
    x2, y2 = (x + 5), (y + 5)
    w.create_oval(x1, y1, x2, y2, width = 0, fill=pythonGreen)

def drawLine(p1,p2,color):
    return(w.create_line(p1[0], p1[1], p2[0], p2[1],fill = color))


identifiers = []
a=[0,0]

def drawRobot(x,y,heading): 
    global identifiers,a
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
    a[1] = (drawLine(scaled[3], scaled[3-1],"#48BEFF"))
    
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


discs = []
def drawDisc(x,y):
    global discs
    x1, y1 = (x - 15.1525), (y - 15.1525)
    x2, y2 = (x + 15.1525), (y + 15.1525)
    discs.append(w.create_oval(x1, y1, x2, y2, width = 0, fill="#FFFC99"))

def getDiscs():
    return discs

def populateField():
    for i in range(5):
        drawDisc(166 + 66 * i, 76 + 66*i)
        drawDisc(826 - 66 * i, 736 - 66 * i)

    for i in range(3):
        offset = 50.8475
        drawDisc(364 - 15.1525 - offset*i, 538 - 15.1525)
        drawDisc(364 + 15.1525, 538 + 15.1525 + offset*i)
        drawDisc(760 - 15.1525 - offset*i, 274 + 15.1525)
        drawDisc(628 - 15.1525, 274 - 15.1525 - offset*i)
        drawDisc(430 + 66*i, 472+66*i)
        drawDisc(562 - 66*i,340 - 66*i)

    drawDisc(298,340)
    drawDisc(594 +100,462+10)

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
canvas_width = 18*40 + 270
canvas_height = 18*40 + 100

master = Tk()
master.title("g")
w = Canvas(master, width=canvas_width, height=canvas_height,background = "#B2DBBF")
#792 x 792
# (100,802)
# (892,10)
# (100,10)
# (892,802)

#i for x
#j for y

colors = ["#6699CC", "#70A6DC"]
for i in range(6):
    for j in range(6):
        cx = 100 + i * 132
        cy = 802 - j * 132
        w.create_rectangle(cx,cy,cx+132,cy-132,fill = colors[j%2 - i%2],outline= "#F45B69")

# w.create_rectangle(100 ,802 ,892 ,10,fill = "#305252") 

for i in range(1,6):
    w.create_line(100,802 - 132*i,892,802 - 132*i, fill = "#F45B69")
    w.create_line(100 + 132*i,802,100+132*i,10, fill = "#F45B69")


#12*12*5.5
w.pack(expand=YES, fill=BOTH)


# drawRobot(path[0][0], path[0][1],0)


# mainloop()