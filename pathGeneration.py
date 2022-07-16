import tkinter
import mainGraphics as mg

    
path = []
points = []
pathIdentifiers =[]
replace = False
prevIndex = 0

def createPath(event):
    global pathIdentifiers

    for i in range(len(pathIdentifiers)):
        mg.w.delete(pathIdentifiers[i])

    
    for i in range(len(points)):
        points[i].place_forget()

    x, y = (event.x), (event.y)
    global replace
    if(not replace):
        path.append((x,y))
        point = tkinter.Button(width = 2, height = 1,command = lambda t = len(path)-1: edit(t), bg= "black")
        points.append(point)
    if(replace):
        path[prevIndex] = (x,y)
        point = tkinter.Button(width = 2, height = 1,command = lambda t = len(path)-1: edit(t), bg= "black")
        points[prevIndex] = (point)
        replace = False

    # print(path)
    for i in range(len(path)):
        pathIdentifiers.append(mg.drawPoint(path[i][0], path[i][1]))
        try:
            pathIdentifiers.append(mg.drawLine((path[i][0], path[i][1]), (path[i+1][0], path[i+1][1]),"black"))
        except IndexError:
            pass


            

def showPoints(event):
    for i in range(len(points)):
        points[i].place(x = path[i][0] - 10,y = path[i][1] - 10)

def edit(t):
    global prevIndex
    global replace
    prevIndex = t
    replace = True

def export():
    name = input("path name ")
    f = open("robotics/" + name + ".txt",'w')
    f.write("coordinate cords[] = {")
    for i in range(len(path)):
        f.write("coordinate(" + str(path[i][0]) + "," + str(path[i][1]) + "), ")
    f.write("};")
    f.close()

    f = open ("robotics/" + name +".keej", 'w')
    for i in range(len(path)):
        f.write(str(path[i]))
    f.close()

exportButton = tkinter.Button(width = 5, height = 2,command = export, bg= "black", text="export")

mg.w.bind("<Button-1>", createPath)
mg.w.bind("<Button-2>", showPoints)   

# message = Label(master, text="Press and Drag the mouse to draw")
# message.pack(side=BOTTOM)