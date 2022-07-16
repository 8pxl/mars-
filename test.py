from math import sqrt
def distToPoint(x,y,px, py):
  return sqrt( (px-x)**2 + (py-y)**2 ) 


print(distToPoint(496,382,410,563))