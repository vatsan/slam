"""
Srivatsan Ramanujam, Nov-2007
Implementation of Particle Filtering algorithm 
for Simultaneous Localization and Mapping (SLAM)
in Autonomous Robots.
"""

import os,sys
from math import sqrt,sin,cos,atan,atan2,fabs,pi
import thread,time
from Tkinter import Canvas,Tk,NW
from PIL import Image,ImageTk
import numpy


Grid = None
cellSize = 0.1
gridSize = 20.0
#identifies which type of update should be used on the SONAR (1 signifies laser, 2 signifies local-maxima(short range) 3 Banana Gaussian
Type = '1'
#increment and decrement values for the cells(best 19,2)
inc = 15
dec = 1
#threshold cell values to range between maxLow and maxHigh, we need to scale it to fit in (0-255) to be displayed as image-pixels
maxLow = -10.0
maxHigh = 200.0
#painting interval (for how many readings should the map be updated)
interval = 20
numParticles=15

numReadings = 0
#used by Tkinter methods for updation of map on canvas
win = None
can = None
img = None

IMG=None

xStart = None
xEnd = None
yStart = None
yEnd = None

#Define Particles
class Particles:
      def __init__(self,numParticles,x,y,theta):
            from random import random
	    self.random = random
	    self.X = [x for k in range(numParticles)]
	    self.Y = [y for k in range(numParticles)]
	    self.Theta = [theta for k in range(numParticles)]
	    self.Weight = [(1.0/numParticles) for k in range(numParticles)]
	    self.numParticles = numParticles
	    from random import gauss,random
	    from math import cos,sin
	    self.gauss = gauss
	    self.random = random
	    self.cos = cos
	    self.sin = sin

      def resampleParticles(self):
	  weightArr=[0 for k in range(self.numParticles)]
	  for k in range(self.numParticles):
              weightArr[k]=sum(self.Weight[:(k+1)])

	  tempX=[0 for k in range(self.numParticles)]
	  tempY=[0 for k in range(self.numParticles)]
	  tempTheta=[0 for k in range(self.numParticles)]
	  tempWeight=[1.0/self.numParticles for k in range(self.numParticles)]

	  for j in range(self.numParticles):
	      sample = self.random()
	      for k in range(self.numParticles):
	          if(sample<weightArr[k]):
	 	      tempX[j],tempY[j],tempTheta[j],tempWeight[j] = self.X[k],self.Y[k],self.Theta[k],self.Weight[k]

	  self.X=tempX
	  self.Y=tempY
	  self.Theta=tempTheta
	  self.Weight=tempWeight
	  self.normalizeWeights()

      def updateParticles(self,deltaS,alpha,deltaPhi):
	  k1=0.03
	  k2=0.3
	  for k in range(self.numParticles):
	      g=self.gauss(0,1.0)
	      #e1=self.gauss(0,k1*alpha)
	      #e2=self.gauss(0,k2*deltaS)
	      #e3=self.gauss(0,k1*(deltaPhi-alpha))
	      e1=g*k1*alpha
	      e2=g*k2*deltaS
	      e3=g*k1*(deltaPhi-alpha)
	      #print '\n# alpha,deltaS,deltaphi,e1,e2,e3:',alpha,deltaS,deltaPhi,e1,e2,e3
	      self.X[k]+=((deltaS+e2)*self.cos(self.Theta[k]+alpha+e1))
	      self.Y[k]+=((deltaS+e2)*self.sin(self.Theta[k]+alpha+e1))
	      self.Theta[k]+=(deltaPhi+e1+e3)
	      #Bound the angle between -pi to pi
	      self.Theta[k] = self.boundAngle(self.Theta[k])
		  
      def boundAngle(self,phi):
	from math import fmod,pi
        #Bound angle to [-pi,pi]
        if (phi >= 0):
            phi = fmod(phi,2*pi);
        else:
            phi = fmod(phi,-2*pi) 
        if (phi > pi):
            phi -= 2*pi
        if (phi < -pi):
            phi += 2*pi

        return phi

      def updateWeights(self,particle,weight):
	  self.Weight[particle] = weight
	  
      def normalizeWeights(self):
	  sumOfWeights = sum(self.Weight)
	  for k in range(self.numParticles):
	      self.Weight[k]/=sumOfWeights

      def selectChampion(self):
	  champion = self.Weight.index(max(self.Weight))
	  return [self.X[champion],self.Y[champion],self.Theta[champion],champion]
      
#Bound the angle in radians between -Pi and Pi
def boundAngle(phi):
    from math import fmod,pi
    #Bound angle to [-pi,pi]
    if (phi >= 0):
        phi = fmod(phi,2*pi);
    else:
        phi = fmod(phi,-2*pi) 
    if (phi > pi):
        phi -= 2*pi
    if (phi < -pi):
        phi += 2*pi

    return phi

#Compute the weighted orientation of two angles
def weightedOrientation(o1,o2,w1,w2):
  # Finds weighted average of o1 and o2 (angles between -pi and pi)
  # Also: w1 + w2 = 1
  from math import pi
  o=0.0

  x1 = cos(o1)
  y1 = sin(o1)
  
  x2 = cos(o2)
  y2 = sin(o2)
  
  x = w1*x1 + w2*x2
  y = w1*y1 + w2*y2
    
  if ( ((fabs(x)<1e-10) and (fabs(y)<1e-10)) or (fabs(fabs(o1-o2)-pi)<1e-10) ):
    o = w1*o1 + w2*o2
  else:
    o = atan2(y,x)

  return o

def readDataFromFile(fileName):
    L = []
    S = []
    O = []
    fl = file(fileName,'r')
    for line in fl:
	if line.startswith('L'):
	   L.append(line[2:len(line)-1].split(' '))
	if line.startswith('S'):
	   S.append(line[2:len(line)-1].split(' '))
	if line.startswith('O'):
	   O.append(line[2:len(line)-1].split(' '))
	 
    return [L,S,O] 
	
def createMap(grid_Size,cell_Size):
    global cellSize 
    cellSize = cell_Size	
    global gridSize
    gridSize = grid_Size
    global Grid 
    global xStart,xEnd,yStart,yEnd
    Grid = [[0.0 for row in range(int(grid_Size/cellSize))] for col in range(int(grid_Size/cell_Size))]
    xStart,xEnd,yStart,yEnd=0,len(Grid),0,len(Grid)
    w,h = len(Grid[0]),len(Grid)
    global img
    img = numpy.zeros((h,w,4), numpy.uint8)

#update beliefs assuming the SONAR is similar to the LASER    
def updateBeliefs(x0,y0,x1,y1):
    global gridSize
    global cellSize
    global inc
    global dec
    global xStart,xEnd,yStart,yEnd
    #find slope of the line
    t=atan2((y1-y0),(x1-x0))
    #find euclidean distance between the two points
    d = sqrt(pow(x1-x0,2)+pow(y1-y0,2))
    # to avoid wasteful computation of updating all grid cells, we will divide the line joining the two points evenly into 
    # d points (d is the length of the line), then along these d points on the line, we will find out which grid it lies on, and update their
    # values.(for simplicity, we will round off the values of these d points to the nearest integral values)
    # we increment x and y in the below loop in proportional to the sine and cosine of the slope of the line, so that the line is EVENLY
    # DIVIDED into d points.

    if(d>49 and (Type=='0' or Type=='-1' or Type=='1')):
	return
    
    for K in range(int(d*10.0)):
	k = K/10.0
	x = int((x0 + ((x1-x0)/fabs(x1-x0))*fabs(k*cos(t)))/cellSize)
	y = int((y0 + ((y1-y0)/fabs(y1-y0))*fabs(k*sin(t)))/cellSize)
	# Now since the Grid is stored as a 2-D array, and its array index starts at (0,0) we shud map the obtained values of x and y 
	# such that x=0,y=0 corresponds to the central field in the 2-D array
	# we apply transform x = x+(gridSize/cellSize*2), y = y+(gridSize/cellSize*2) such that dimensions of the array gridSize/cellSize)
	
	#the map appears to be more prominent in the lower half of Y-plane of the image, so let us shift X-axis further, 
        #so that the map is more visible (note X-axis in the array is the Y axis in the image.
	x += int(gridSize/(cellSize*6))
	y += int(gridSize/(cellSize*2))
	if(k==0.0 and x>=0 and x<int(gridSize/cellSize) and y>=0 and y<int(gridSize/cellSize)):
	    if(x<xStart):
		xStart=x
	    if(y<yStart):
		yStart=y	
	    if(x>xEnd):
	        xEnd=x
	    if(y>yEnd):
	        yEnd=y
	
	# decrement the value in the corresponding cell (if it is within our map)
	if(x>=0 and x<int(gridSize/cellSize) and y>=0 and y<int(gridSize/cellSize) and k >0.0):
	    Grid[x][y] -= dec


    #increment the occupancy belief of the cell lying on the endpoint of the laser
    x = int(round(x1/cellSize))
    y = int(round(y1/cellSize))
    #transform to proper coordinates of array
    #the map appears to be more prominent in the lower half of Y-plane of the image, so let us shift X-axis further, 
    #so that the map is more visible (note X-axis in the array is the Y axis in the image.
    x += int(gridSize/(cellSize*6))
    y += int(gridSize/(cellSize*2))
    # increase occupancy belief if the points lie within the map
    if(x>=0 and x<int(gridSize/cellSize) and y>=0 and y<int(gridSize/cellSize)):
	if(x>xEnd):
	    xEnd=x
	if(y>yEnd):
	    yEnd=y
	if(x<xStart):
	    xStart=x
	if(y<yStart):
	    xStart=y
        Grid[x][y] += inc

	
#Use the Banana Shaped Gaussian Distribution to decide which cells to update and by how much.
def updateBeliefsBananaGaussian(x0,y0,x1,y1):
    from math import radians
    global gridSize
    global cellSize
    global inc
    global dec
    x11 = x1
    y11 = y1
    THETA = atan2((y1-y0),(x1-x0))
    D = sqrt(pow(x1-x0,2)+pow(y1-y0,2))
    #ignore long range returns, they are likely to be specular reflections
    if(D>12):
	return
    #the chirp could extend upto a 30 degree angle, 12 degrees to the either side of the central beam
    for theta in range(-12,12):
	angle =  THETA+radians(theta)
	x11 =  x0 + ((x1-x0)/fabs(x1-x0))*fabs(D*cos(angle))
	y11 =  y0 + ((y1-y0)/fabs(y1-y0))*fabs(D*sin(angle))
        #find slope of the line
        t=atan2((y11-y0),(x11-x0))
        #find euclidean distance between the two points
        d = sqrt(pow(x11-x0,2)+pow(y11-y0,2))
        for K in range(int(d*10.0)):
	    k = K/10.0
	    x = int((x0 + ((x11-x0)/fabs(x11-x0))*fabs(k*cos(t)))/cellSize)
	    y = int((y0 + ((y11-y0)/fabs(y11-y0))*fabs(k*sin(t)))/cellSize)

	    x += int(gridSize/(cellSize*6))
	    y += int(gridSize/(cellSize*2))
	
            # decrement the value in the corresponding cell (if it is within our map)
	    if(x>=0 and x<int(gridSize/cellSize) and y>=0 and y<int(gridSize/cellSize) and k >0):
		# decrements will be proporational to r,theta (to obtain bivariate gaussian, we multiply the values frm the  
		# two univariate gaussians.
	        Grid[x][y] -= (0.4*dec+dec*(Gaussian(D)*Gaussian(radians(theta))))

        #increment the occupancy belief of the cell lying on the endpoint of the SONAR Chirp
        x = int(round(x11/cellSize))
        y = int(round(y11/cellSize))
        #transform to proper coordinates of array
        #the map appears to be more prominent in the lower half of Y-plane of the image, so let us shift X-axis further, 
        #so that the map is more visible (note X-axis in the array is the Y axis in the image.
        x += int(gridSize/(cellSize*6))
        y += int(gridSize/(cellSize*2))
        # increase occupancy belief if the points lie within the map
        if(x>=0 and x<int(gridSize/cellSize) and y>=0 and y<int(gridSize/cellSize)):
	    # increments will be proporational to r,theta (to obtain bivariate gaussian, we multiply the values frm the  
	    # two univariate gaussians.	
            Grid[x][y] += (0.4*inc + inc*(Gaussian(D)*Gaussian(radians(theta))))	
	
def Gaussian(x):
    x = float(x)	
    #this corresponds to a short range sonar(it cannot be a radian value as that range from (-.26 to .26), so let us give it
    #max-return from the Gaussian, it is unrealistic to expect
    #sonars with range ~0, for the Gaussian to make sense.
    if(x>0.3 and x<6.0):
	x = 0.0
    from math import pi,exp
    #define a gaussian with mean 0 and unit variance. Return the value of the function at 'x'
    #1) this corresponds to the Gaussian over the Theta value of the SONAR sample, the more perpendicular the return, higher the value.
    #2) this also corresponds to the Gaussian over the 'r' value, if r is zero, the return is maximum, higher values of r will fetch
    # lower returns.
    #3) To obtain a bivariate gaussian in r,theta, simply multiply the return values from this function, one for 'r', the other for theta.
    return ((1.0/sqrt(2.0*pi))*exp(-x*x/2.0))	
	
#Use only short range Sonar returns for updation of beliefs, longer range returns of Sonar chirp will be ignored.	
def updateBeliefsShortRange(x0,y0,x1,y1):
    global gridSize
    global cellSize
    global inc
    global dec
    #find slope of the line
    t=atan2((y1-y0),(x1-x0))
    #find euclidean distance between the two points
    d = sqrt(pow(x1-x0,2)+pow(y1-y0,2))
    # to avoid wasteful computation of updating all grid cells, we will divide the line joining the two points evenly into 
    # d points (d is the length of the line), then along these d points on the line, we will find out which grid it lies on, and update their
    # values.(for simplicity, we will round off the values of these d points to the nearest integral values)
    # we increment x and y in the below loop in proportional to the sine and cosine of the slope of the line, so that the line is EVENLY
    # DIVIDED into d points.

    #Disregard longer SONAR/Laser returns, they are likely due to specular reflections.
    if(d>15.0):
       return 

    for K in range(int(d*10.0)):
	k = K/10.0
	x = int((x0 + ((x1-x0)/fabs(x1-x0))*fabs(k*cos(t)))/cellSize)
	y = int((y0 + ((y1-y0)/fabs(y1-y0))*fabs(k*sin(t)))/cellSize)
	# Now since the Grid is stored as a 2-D array, and its array index starts at (0,0) we shud map the obtained values of x and y 
	# such that x=0,y=0 corresponds to the central field in the 2-D array
	# we apply transform x = x+(gridSize/cellSize*2), y = y+(gridSize/cellSize*2) such that dimensions of the array gridSize/cellSize)
	
	#the map appears to be more prominent in the lower half of Y-plane of the image, so let us shift X-axis further, 
        #so that the map is more visible (note X-axis in the array is the Y axis in the image.
	x += int(gridSize/(cellSize*6))
	y += int(gridSize/(cellSize*2))
	
	# decrement the value in the corresponding cell (if it is within our map)
	if(x>=0 and x<int(gridSize/cellSize) and y>=0 and y<int(gridSize/cellSize) and k >0):
	    Grid[x][y] -= dec
	
    #increment the occupancy belief of the cell lying on the endpoint of the laser
    x = int(round(x1/cellSize))
    y = int(round(y1/cellSize))
    #transform to proper coordinates of array
    #the map appears to be more prominent in the lower half of Y-plane of the image, so let us shift X-axis further, 
    #so that the map is more visible (note X-axis in the array is the Y axis in the image.
    x += int(gridSize/(cellSize*6))
    y += int(gridSize/(cellSize*2))
    # increase occupancy belief if the points lie within the map
    if(x>=0 and x<int(gridSize/cellSize) and y>=0 and y<int(gridSize/cellSize)):
        Grid[x][y] += inc	
	
def magnifyGrid(magnifyFactor):
    mgGrid = [[0 for col in range(len(Grid)*magnifyFactor)] for row in range(len(Grid)*magnifyFactor)]
    
    for x in range(len(Grid)):
	for y in range(len(Grid[0])):
	    for p in range(magnifyFactor):
		for q in range(magnifyFactor):
		    mgGrid[x*magnifyFactor+p][y*magnifyFactor+q] = Grid[x][y]

    return mgGrid
   
def writeImage(grid,fileName=None):
    global maxLow,maxHigh
    global xStart,xEnd,yStart,yEnd
    #import numpy

    global img
    if(img==None):
	w,h = len(grid[0]),len(grid)
        img = numpy.zeros((h,w,4), numpy.uint8)

    if(fileName):
        xStart,xEnd,yStart,yEnd=0,len(grid),0,len(grid)

    xStart-=10
    xEnd+=10
    yStart-=10
    yEnd+=10
    if(xStart<0):
	xStart=0
    if(xEnd>len(Grid)):
	xEnd=len(Grid)
    if(yStart<0):
	yStart=0
    if(yEnd>len(Grid)):
	yEnd=len(Grid)
	
    #print '\n #### xStart,xEnd,yStart,yEnd:',xStart,xEnd,yStart,yEnd
    xStart,xEnd,yStart,yEnd=0,len(grid),0,len(grid)

    for k in range(xStart,xEnd):
	for l in range(yStart,yEnd):
            belief = grid[k][l]
	    red = 0
	    blue = 0
	    green = 0
	    # such high negative values of the grid cells mostly indicate the robot was present in this cell at some point.
	    # coz we had set the cell value to -999999 then.
	    if(belief < - 666666):
		red = 0
		green = 0
		blue = 255
	    if(belief > -666666 and belief < 0):
	        red = 255
		blue = 255
		green = 255
	    if(belief == 0.0):
		red = 170
		blue = 170
		green = 170
	    if(belief > 30.0):
		red = 0
		blue = 0
		green = 0

	    img[k][l] = [red,green,blue,255]

    pilImage = Image.fromarray(img,'RGBA')

    if(fileName!=None):
        pilImage.save(fileName+'.png') 
    else:   
        im = ImageTk.PhotoImage(pilImage)
	showMap(im)

def getInterpolatedOdometry(sonarCol,rawOdometry):
    correctedOdo = []
    #initX,initY,initTheta=float(rawOdometry[0][1]),float(rawOdometry[0][2]),float(rawOdometry[0][3])
    initX,initY,initTheta=0.0,0.0,0.0
    for k in range(len(sonarCol)-1):
	#difference in time stamps of Sonar/Laser and raw odometry
	deltaT = float(sonarCol[k][0]) - float(rawOdometry[k][0])
	#time difference between two consecutive odometry readings
	T = float(rawOdometry[k+1][0])-float(rawOdometry[k][0])
	#use linear interpolation to obtain corrected Odometry, wrt to time stamp of sonar
	# interpolated value of x coordinate
	x = float(rawOdometry[k][1]) + ((float(rawOdometry[k+1][1]) - float(rawOdometry[k][1]))/T)*deltaT
	#interpolated value of y coordinate
	y = float(rawOdometry[k][2]) + ((float(rawOdometry[k+1][2]) - float(rawOdometry[k][2]))/T)*deltaT
	#interpolated value of theta 
	theta = float(rawOdometry[k][3]) + ((float(rawOdometry[k+1][3]) - float(rawOdometry[k][3]))/T)*deltaT
	t=float(sonarCol[k][0])
	t1=float(rawOdometry[k][0])
	t2=float(rawOdometry[k+1][0])
	w1=(t-t1)/(t2-t1)
	w2=1.0-w1
	if(abs(float(rawOdometry[k+1][3]) - float(rawOdometry[k][3]))>pi):
	     theta=float(rawOdometry[k+1][3])
	#theta = weightedOrientation(float(rawOdometry[k][3]),float(rawOdometry[k+1][3]),w1,w2)
	theta = boundAngle(theta)
	correctedOdo.append([sonarCol[k][0],x-initX,y-initY,theta-initTheta])
	
    #simply append the last entry of the raw odometry to the corrected odometry, this is is assuming the robot has stopped.
    correctedOdo.append(rawOdometry[len(rawOdometry)-1])	
	
    return correctedOdo	


def main(datafile='unlocalized.data',cell_Size=0.1,Typ=1):
    global Type
    Type = Typ
    [L,S,O] = readDataFromFile(datafile)	
    global numReadings
    global cellSize
    global xStart,xEnd,yStart,yEnd
    numReadings = len(L)
    print '\nNumber of samples:',len(L)
    gridSize = 50.0
    cellSize = float(cell_Size)/100
    createMap(gridSize,cellSize)

    Onew = None
    if(Type=='0' or Type=='-1'):
	 Onew = getInterpolatedOdometry(L,O)
    else:
	 Onew = getInterpolatedOdometry(S,O)
	
    #initialize the canvas in a new thread	
    thread.start_new_thread(initializeCanvas,())

    #Create the Particle Filter
    particlesObj = Particles(numParticles,Onew[0][1],Onew[0][2],Onew[0][3])

    for k in range(len(S)):
	#discarding time-stamps
	print '\n Reading:',k
	Snew = None
	if(Type=='0' or Type=='-1'):
            Snew = L[k][1:]
	else:
	    Snew = S[k][1:] 

	# Data from the odometry to compute coordinates of Sonar chirp/Laser beam in absolute scale.
	c_x = float(Onew[k][1])
	c_y = float(Onew[k][2])
	theta = boundAngle(float(Onew[k][3]))
	
	#Use the Particle Filter to obtain the Corrected Odometry
	if(k>0 and Type!='-1'):
	    (x00,y00,theta00) = (float(Onew[k-1][1]),float(Onew[k-1][2]),boundAngle(float(Onew[k-1][3])))
	    #print '\n #Current Odometry:',float(O[k][1]),float(O[k][2]),boundAngle(float(O[k][3]))
	    #print '\n #Estimated odometry',float(Onew[k][1]),float(Onew[k][2]),boundAngle(float(Onew[k][3]))
            (c_x,c_y,theta) = localizer(particlesObj,x00,y00,theta00,c_x,c_y,theta,Snew)
	    #print '\n #Corrected Odometry:',c_x,':',c_y,':',theta
	    
	#### CHANGE THE VALUES OF xStart,XEnd,yStart,yEnd for each cycle of Laser/Sonar readings.#####
	if(k>0):
	    xStart=xEnd
	    xEnd=0
	    yStart=yEnd
	    yEnd=0

	for l in range(0,len(Snew),4):
	    x0 = float(Snew[l])
	    y0 = float(Snew[l+1])
	    x1 = float(Snew[l+2])
	    y1 = float(Snew[l+3])
            x0,y0=0.0,0.0
	    #transform the data from robots coordinate to global coordinate
	    # d is the euclidean distance between the points.
	    d = sqrt(pow((x0-x1),2)+pow((y0-y1),2))
	    #find slope of the line joining origin of the robot coordinates to the point reflecting the laser
	    alpha1 = atan2(y1,x1)
	    #location of end points of the SONAR/Laser in the global coordinates 
	    G_x1 = d*cos(theta+alpha1)+c_x
	    G_y1 = d*sin(theta+alpha1)+c_y
	    ########################## SET x0,y0 to the current coordinates of the robot.
	    x0 = c_x
	    y0 = c_y
	    ##########################
            #location of the start point of the SONAR/Laser in global coordinates
	    d0 = sqrt(pow((x0-c_x),2)+pow((y0-c_y),2))
	    #find slope of the line joining origin of the robot coordinates to the starting point of the laser
	    # to avoid divide by zero error, x having 0, with a very small value close to 0
	    alpha0 = atan2(y0,x0)
	    G_x0 = d0*cos(theta+alpha0)+c_x
	    G_y0 = d0*sin(theta+alpha0)+c_y

	    if(Type=='1' or Type=='0' or Type=='-1'):
	       updateBeliefs(G_x0,G_y0,G_x1,G_y1)
	    if(Type=='2'):
	       updateBeliefsShortRange(G_x0,G_y0,G_x1,G_y1)
	    if(Type=='3'):
	       updateBeliefsBananaGaussian(G_x0,G_y0,G_x1,G_y1)

	#Mark the current position of the Robot on the grid(after transforming coordinates to indexes on the grid)
	xloc = c_x #float(Onew[k][1])
	yloc = c_y #float(Onew[k][2])
	x00 = int(round(xloc/cellSize)) + int(gridSize/(cellSize*6))
        y00 = int(round(yloc/cellSize)) + int(gridSize/(cellSize*2))

        #let us add a very high negative value to the cell in which the robot is currently present
        # this will be used to show the path the robot took, during map display.    
        if(x00>=0 and x00<int(gridSize/cellSize) and y00>=0 and y00<int(gridSize/cellSize)):
            Grid[x00][y00] = -999999           

	if(k%interval==0):
	    #mgGrid = magnifyGrid(2)
	    print 'Map:',str(k/interval)
	    writeImage(Grid)#,'Maps/Map'+str(10000+(k/interval))) 

    #magnify the grid so that it is more visible to the eyes.
    #mgGrid = magnifyGrid(2)
    mgGrid=Grid

    mp = 'Maps/FinalMap-'
    if(Type=='-1'):
	mp+='EstimatedOdo-Laser'
    if(Type=='0'):
	mp+='CorrectOdo-Laser'
    if(Type=='1'):
	mp+='CorrectOdo-Sonar-LongRange'
    if(Type=='2'):
	mp+='CorrectOdo-Sonar-ShortRange'
    if(Type=='3'):
	mp+='CorrectOdo-Sonar-Banana-Gaussian'

    writeImage(mgGrid,mp)

#Localizer method, handles all localization
def localizer(particlesObj,x0,y0,phi0,x1,y1,phi1,sensorReadings):
    delx=x1-x0
    dely=y1-y0
    delphi=phi1-phi0
    delphi=boundAngle(delphi)
    deltaS=pow(delx*delx+dely*dely,0.5)
    alpha=0.0
    if(dely==delx==delphi==0.0):
	alpha=0.0
    else:
        alpha=-atan2(dely,delx)+phi0

    particlesObj.updateParticles(deltaS,alpha,delphi)

    #Reweight the particles
    for k in range(particlesObj.numParticles):
       weight = computeWeight(particlesObj,k,sensorReadings)
       #print '\n Weight:',weight
       particlesObj.updateWeights(k,weight)

    #Normalize the weights to lie between 0.0 and 1.0
    particlesObj.normalizeWeights()

    #Use the pose of the champion of the current generation.
    [x,y,theta,bestParticle] = particlesObj.selectChampion()
    #Seed the next generation of particles by probablistic resampling of those from the current generation
    particlesObj.resampleParticles()
    return (x,y,theta)

#used to compute and update the weights of each particle
def computeWeight(particlesObj,k,sensorReadings):
    c_x,c_y,theta=particlesObj.X[k],particlesObj.Y[k],particlesObj.Theta[k]
    weight=0.0
    for k in range(0,len(sensorReadings),4):
	x0 = float(sensorReadings[k])
	y0 = float(sensorReadings[k+1])
	x1 = float(sensorReadings[k+2])
	y1 = float(sensorReadings[k+3])
        x0,y0=0.0,0.0
	#transform the data from robots coordinate to global coordinate
	# d is the euclidean distance between the points.
	d = sqrt(pow((x0-x1),2)+pow((y0-y1),2))
	
	#find slope of the line joining origin of the robot coordinates to the point reflecting the laser
	alpha1 = atan2(y1,x1)
	#location of end points of the SONAR/Laser in the global coordinates 
	G_x1 = d*cos(theta+alpha1)+c_x
	G_y1 = d*sin(theta+alpha1)+c_y
	########################## SET x0,y0 to the current coordinates of the robot.
	x0 = c_x
	y0 = c_y
	##########################
        #location of the start point of the SONAR/Laser in global coordinates
	d0 = sqrt(pow((x0-c_x),2)+pow((y0-c_y),2))
	#find slope of the line joining origin of the robot coordinates to the starting point of the laser
	alpha0 = atan2(y0,x0)
	G_x0 = d0*cos(theta+alpha0)+c_x
	G_y0 = d0*sin(theta+alpha0)+c_y
	
	t=atan2((G_y1-G_y0),(G_x1-G_x0))
        #find euclidean distance between the two points
        d = sqrt(pow(G_x1-G_x0,2)+pow(G_y1-G_y0,2))
	#increment the occupancy belief of the cell lying on the endpoint of the laser
        x = int(round(G_x1/cellSize))
        y = int(round(G_y1/cellSize))
        #transform to proper coordinates of array
        #the map appears to be more prominent in the lower half of Y-plane of the image, so let us shift X-axis further, 
        #so that the map is more visible (note X-axis in the array is the Y axis in the image.
        x += int(gridSize/(cellSize*6))
        y += int(gridSize/(cellSize*2))
        # increase occupancy belief if the points lie within the map
	ON=0
        if(x>=0 and x<int(gridSize/cellSize) and y>=0 and y<int(gridSize/cellSize)):
	     #Ray Terminates exactly on an obstacle
	     for q in [-1,0,1]:
		 for r in [-1,0,1]:
	           if(x+r>0 and x+r<len(Grid) and y+q>0 and y+q<len(Grid)):
	               if(Grid[x+r][y+q]>30.0):
		           weight+=(12.0)
		           ON=1

	OUTSIDE=0
	if(ON==0):
	  for K in range(int(d*1.0)):
	    k = K/1.0
	    x = int((G_x0 + ((G_x1-G_x0)/fabs(G_x1-G_x0))*fabs(k*cos(t)))/cellSize)
	    y = int((G_y0 + ((G_y1-G_y0)/fabs(G_y1-G_y0))*fabs(k*sin(t)))/cellSize)
	    # Now since the Grid is stored as a 2-D array, and its array index starts at (0,0) we shud map the obtained values of x and y 
	    # such that x=0,y=0 corresponds to the central field in the 2-D array
	    # we apply transform x = x+(gridSize/cellSize*2), y = y+(gridSize/cellSize*2) such that dimensions of the array gridSize/cellSize)
	
  	    #the map appears to be more prominent in the lower half of Y-plane of the image, so let us shift X-axis further, 
            #so that the map is more visible (note X-axis in the array is the Y axis in the image.
	    x += int(gridSize/(cellSize*6))
	    y += int(gridSize/(cellSize*2))
	
	    # decrement the value in the corresponding cell (if it is within our map)
	    if(x>=0 and x<int(gridSize/cellSize) and y>=0 and y<int(gridSize/cellSize) and k >0):
		#Does Ray Terminate After an obstacle?
		if(Grid[x][y]>30.0):
		    weight+=(4.0)
		    OUTSIDE=1
		    break
		
        #increment the occupancy belief of the cell lying on the endpoint of the laser
        #x = int(round(G_x1/cellSize))
        #y = int(round(G_y1/cellSize))
        #transform to proper coordinates of array
        #the map appears to be more prominent in the lower half of Y-plane of the image, so let us shift X-axis further, 
        #so that the map is more visible (note X-axis in the array is the Y axis in the image.
        #x += int(gridSize/(cellSize*6))
        #y += int(gridSize/(cellSize*2))
        # increase occupancy belief if the points lie within the map
        #if(x>=0 and x<int(gridSize/cellSize) and y>=0 and y<int(gridSize/cellSize) and OUTSIDE==0):
	     #Ray Terminates exactly on an obstacle
	     #if(Grid[x][y]>30.0):
		#weight+=(12.0)
	     #Ray has terminated before an obstacle
	     #else:
		#weight+=(8.0)
		
        #Ray has terminal before the obstacle
	if(OUTSIDE==0):
		weight+=8.0
		
    return weight
	
	
#this method will be called in a new thread as well, to initialize the canvas
def initializeCanvas():
    global can,win	
    win = Tk()
    win.title('vatsan SLAM')
    can = Canvas(win, width=gridSize/cellSize,height=gridSize/cellSize)
    can.pack() 
    #start the showMap method is a new thread
    thread.start_new_thread(showMap,(None,))
    win.mainloop()

#this function will be called in a new thread, so that it continuously draws the changing map on the canvas    
def showMap(img1):
    global IMG
    IMG = img1
    if(img1 != None):
        can.create_image(0,0,anchor=NW,image = img1)

if(__name__=='__main__'):
    if not sys.argv[1:] or len(sys.argv)<4:
        print "Usage: vatsanMap.py <datafile [ex:localized.data]> <cell size [ex:20]> <Type (-1/0/1/2,3) [ex:0]>"
        sys.exit()
		
    datafile = sys.argv[1]
    cellSize = sys.argv[2]	
    Type = sys.argv[3]	  
    main(datafile,float(cellSize),Type)

