#!/usr/bin/env python
import roslib
import rospy, tf
import math
import time 
from kobuki_msgs.msg import BumperEvent
#
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import degrees
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D
from kobuki_msgs.msg import BumperEvent 
# Add additional imports for each of the message types used
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
from std_msgs.msg import Header
from std_msgs.msg import Float64
class Node:
    def __init__(self, x, y, theta):
        self.z = 0
        self.x = x
        self.y = y
        self.theta = theta % 360
        self.radian = theta*math.pi/180
class Cost:
    def __init__(self, costSoFar,heuristic):
        self.g = costSoFar
        self.h = heuristic
        self.f = costSoFar + heuristic
class ClosedNode:
    def __init__(self, node, costSoFar, backNode):
        self.g = costSoFar
        self.node = node
        self.path = backNode
class Frontier:
    def __init__(self, node, cost, path):
        self.node = node#this is a Node
        self.cost = cost#this is a Cost
        self.path = path# is the past new
global disPerGrid#constant on the length of gridcells
disPerGrid = .2
global disPerdeg #constant on the length traveled per degrees spun
disPerdeg = .23*3.14/360
def aStar(rmap, start, end):# example call aStar(testmap, Node(4,0,46), Node(2,0,100))
    frontierList = list()
    closedList =list()
    startGrid= startToGrid(start)
    startOfAllList = 0

    frontierList.append(Frontier(startGrid,Cost(0, calculateHeuristic(startGrid, end)),startOfAllList))
    error = 0
    done = 0
    if(rmap[start.x][start.y]==100 or rmap[end.x][end.y] == 100):
        error = 1

    while(done == 0 and error == 0):
        currentNode = frontierList.pop(0)
        print"x: %f y: %f t: %f" % (currentNode.node.x, currentNode.node.y, currentNode.node.theta)
        neighborNodes =[]
        if(abs(currentNode.node.theta) <= 22.5 or abs(currentNode.node.theta) >= 337.5):
            dx=1
            dy=0
        elif(abs(currentNode.node.theta) <= 67.5 and abs(currentNode.node.theta) >= 22.5):
            dx=1
            dy=1
        elif(abs(currentNode.node.theta) <= 112.5 and abs(currentNode.node.theta) >= 67.5):
            dx=0
            dy=1
        elif(abs(currentNode.node.theta) <= 157.5 and abs(currentNode.node.theta) >= 112.5):
            dx=-1
            dy=1
        elif(abs(currentNode.node.theta) <= 202.5 and abs(currentNode.node.theta) >= 157.5):
            dx=-1
            dy=0
        elif(abs(currentNode.node.theta) <= 247.5 and abs(currentNode.node.theta) >= 202.5):
            dx=-1
            dy=-1
        elif(abs(currentNode.node.theta) <= 292.5 and abs(currentNode.node.theta) >= 247.5):
            dx=0
            dy=-1
        else:
            dx=1
            dy=-1
        neighborNodes.append(Node(currentNode.node.x, currentNode.node.y, (currentNode.node.theta + 45)))
        neighborNodes.append(Node(currentNode.node.x, currentNode.node.y, (currentNode.node.theta - 45)))
        newx =currentNode.node.x+dx
        newy =currentNode.node.y+dy
        if(not(newx < 0 or newx >= len(rmap) or newy < 0 or newy >= len(rmap[0])) and (rmap[newx][newy]==0) and (rmap[newx-dx][newy]==0) and (rmap[newx][newy-dy]==0)):
            neighborNodes.append(Node(currentNode.node.x+dx, currentNode.node.y+dy, currentNode.node.theta))
        for node in neighborNodes:
            closedList = updateClosedList(closedList, node,currentNode.cost.g+changeInNode(currentNode.node, node), currentNode.path )
            if(not(nodeInList(closedList, node))):
                newCost = Cost(currentNode.cost.g+changeInNode(currentNode.node, node),calculateHeuristic(node, end))
                newClosedNode = ClosedNode(currentNode.node,currentNode.cost.g,currentNode.path)
                frontierList = addFrontier(frontierList, Frontier(node, newCost, newClosedNode))
        closedList.append(newClosedNode)
        if(len(frontierList) == 0):
            error = 1
        elif(frontierList[0].node.x==end.x and frontierList[0].node.y==end.y):
            done =1
       

    finalPath = []
    if(error == 1):
        finalPath.append(start)
    else:
        finalFrontier = frontierList.pop(0)
        finalNode = finalFrontier.path
        while(not(finalNode.path == 0)):
            finalPath.insert(0, finalNode.node)
            finalNode = finalNode.path
        finalPath.insert(0, start)
    return finalPath
def printPath(path):
    step = 1
    for n in path:
        print"step %f x: %f y: %f t: %f" % (step, n.x, n.y, n.theta)
        step = step + 1

def nodeInList(nodeList, node):
    isIn = 0
    for n in nodeList:
        if(n.node.x == node.x and n.node.y == node.y and n.node.theta == node.theta):
            isIn = 1
    return isIn
def updateClosedList(closedList, node, g, path):
    insert = -1
    counter = 0
    for n in closedList:
        if(n.node.x == node.x and n.node.y == node.y and n.node.theta == node.theta):
            if(n.g > g):
                insert = counter
        counter += 1
    if(insert>-1):
        closedList[insert].path = path
        closedList[insert].g = g
        print "cooo"
    return closedList
def changeInNode(startNode, endNode):
    dx = endNode.x - startNode.x
    dy = endNode.y - startNode.y
    dtheta = (endNode.theta - startNode.theta + 360)%360
    if(dtheta>180):
        dtheta=360 - dtheta
    return (((dx**2 + dy**2)**.5)*disPerGrid+abs(dtheta*disPerdeg))
def addFrontier(frontierList, frontier):
    i = 0# what index it is in the list
    insert = -1#where it will insert the frontier
    smaller = 1# if the new frontier is smaller than any of the same frontiers
    pop = -1
    for f in frontierList:

        if ((frontier.cost.f < f.cost.f) and insert == -1):
            insert = i
        if (nodeEqual(frontier.node, f.node) and (frontier.cost.f < f.cost.f)):
            pop = i
        if (nodeEqual(frontier.node, f.node) and not(frontier.cost.f < f.cost.f)):
            smaller = 0
    if(pop>-1):
        frontierList.pop(pop)
    if(smaller == 1):
        if(insert>-1):
            frontierList.insert(insert, frontier)
        else:
            frontierList.append(frontier)
    
    return frontierList
def nodeEqual(nodeA, nodeB):
    if(nodeA.x == nodeB.x and nodeA.y == nodeB.y and nodeA.theta == nodeB.theta):
        return 1
    else:
        return 0

def startToGrid(start):
    if(abs(start.theta) <= 22.5 or abs(start.theta) >= 337.5):
        return Node(start.x, start.y, 0)
    elif(abs(start.theta) <= 67.5 and abs(start.theta) >= 22.5):
        return Node(start.x, start.y, 45)
    elif(abs(start.theta) <= 112.5 and abs(start.theta) >= 67.5):
        return Node(start.x, start.y, 90)
    elif(abs(start.theta) <= 157.5 and abs(start.theta) >= 112.5):
        return Node(start.x, start.y, 135)
    elif(abs(start.theta) <= 202.5 and abs(start.theta) >= 157.5):
        return Node(start.x, start.y, 180)
    elif(abs(start.theta) <= 247.5 and abs(start.theta) >= 202.5):
        return Node(start.x, start.y, 225)
    elif(abs(start.theta) <= 292.5 and abs(start.theta) >= 247.5):
        return Node(start.x, start.y, 270)
    else:
        return Node(start.x, start.y, 315)
def calculateHeuristic(node, goal):
    dx = (goal.x - node.x)
    dy = (goal.y - node.y)
    if(dy == 0 and dx == 0):
        return 0
    elif (dx == 0):
        theta = 90*(abs(dy)/dy)
    else:
        theta = math.atan(dy/dx)*180/3.14
    if(dx < 0):
        theta += 180
    theta = (theta + 360)%360
    if((theta-node.theta+360)%360>180):
        dTheta=360-((theta-node.theta+360)%360)
    else:
        dTheta=((theta-node.theta+360)%360)
    return (((dx**2 + dy**2)**.5)*disPerGrid+dTheta*disPerdeg)
def generateWaypoints(nodes):
    waypoints = []
    lastNode = nodes[0]
    waypoints.append(lastNode)
    i=0
    for node in nodes:
       if( (not(lastNode.x == node.x)or not(lastNode.y == node.y)) and not(waypoints[len(waypoints)-1].theta == lastNode.theta)):
           waypoints.append(lastNode)
       lastNode = Node(node.x, node.y, node.theta)
    waypoints.append(nodes[len(nodes)-1])
    waypoints.pop(0)
    return waypoints
def listToMapYX(rlist, width, hieght):
    rmap = []
    workingList = []
    workingList.extend(rlist)
    counter = 1
    rowList =[]
    rowList.append(workingList.pop(0))
    for obj in workingList:
       if (counter % hieght == 0):
           rmap.append(rowList)
           rowList = []
       rowList.append(workingList.pop(0))
       counter += 1
    return rmap
def listToMapXY(rlist, width, height):
    rmap = []
    ycounter = 0
    rowList =[]
    while(ycounter < width):
        xcounter = 0
        while(xcounter + ycounter < len(rlist)):
            rowList.append(rlist[xcounter + ycounter])
            xcounter += width
        rmap.append(rowList)
        rowList =[]
        ycounter += 1
    return rmap
def gridNodesToMeter(nodeList, xOffset, yOffset, rez):
    workingList =[]
    workingList.extend(nodeList)
    for node in workingList:
        node.x = (node.x*rez+xOffset)
        node.y = (node.y*rez+yOffset)
    return workingList
# example of everything all together
#printPath(generateWaypoints(aStar(listToMapXY(rvizmap, 37, 37), Node(1,1,46), Node(35,35,0))))
# ^ prints steps out to console
#               ^ takes all points of path a sorts out only point that are improtant i.e. when turning is required
#                              ^finds path and returns a list of nodes of all nodes traveled
#                                        ^ turns a map in list form into a 2d map
#                                                                     ^start node (x, y, theta)
#                                                                                  ^ end node (x, y, theta)
#This function sequentially calls methods to perform a trajectory.

def executeTrajectory():

	#time.sleep(.1)
	#while(bumper==0):
		#checkBumper()	
		#time.sleep(.05)
	driveStraight(.2,.6)
	rotate(-3.14/2)
	driveArc(-.15,.1,3.14)
        rotate(3*3.14/4)
	driveStraight(.2,.42)






#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, t):
	#wheel diameter is .076 meters and wheel base is 
	u = (u1+u2)/2
	w = (u2-u1)/(.230)
	start =time.time()
	while(time.time() - start < t):
		publishTwist(u,w)
	#publishTwist(0,0)


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	poleRate = .1
	tol = .1
	#getOdom()
	time.sleep(.5)
	
	xStart = x
	yStart = y
	print "x: %f" % (x)
	print "y: %f" % (y)
	xGoal =xStart + distance*math.cos(theta)
	yGoal = yStart + distance*math.sin(theta)
	print "xGoal: %f" % (xGoal)
	print "yGoal: %f" % (yGoal)
	while((x < xGoal -tol or x > xGoal+tol) or (y < yGoal -tol or y > yGoal+tol)):
		publishTwist(speed, 0)
		getOdom()
		print "x: %f" % (x)
		print "y: %f" % (y)
		
		f.write(str.format("{0:.3f}", x))
		f.write(",")
		f.write(str.format("{0:.3f}", y))
		f.write("\n")
		time.sleep(poleRate)
	publishTwist(0,0)


    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
	poleRate = .1
	tol = .15
	#getOdom()
	time.sleep(.5)
	
	thetaStart = theta
	
	print "theta: %f" % (theta)
	
	thetaGoal = (((thetaStart+angle)+3.14)%(2*3.14))-3.14
	print "thetaGoal: %f" % (thetaGoal)
	
	while(theta < thetaGoal -tol or theta > thetaGoal+tol):
		publishTwist(0, angle/abs(angle))
		#getOdom()
		print "theta: %f" % (theta)
		
		
		f.write(str.format("{0:.3f}", x))
		f.write(",")
		f.write(str.format("{0:.3f}", y))
		f.write("\n")
		time.sleep(poleRate)
	publishTwist(0,0)



#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	poleRate = .1
	tol = .1
	#getOdom()
	time.sleep(.5)
	wheelToCenter = .23/2
	
	thetaStart = theta
	
	print "theta: %f" % (theta)
	
	thetaGoal = (((thetaStart+angle)+3.14)%(2*3.14))-3.14
	print "thetaGoal: %f" % (thetaGoal)
	if(radius == 0):
		leftmotor=-.1*angle/abs(angle)
		rightmotor=.1*angle/abs(angle)
	elif(radius == wheelToCenter):
		leftmotor=0
		rightmotor=2*speed
	elif(radius == -1*wheelToCenter):
		leftmotor=2*speed
		rightmotor=0
	else:
		leftmotor = 2*speed/((radius+wheelToCenter)/(radius-wheelToCenter)+1)
		rightmotor =  2*speed/((radius-wheelToCenter)/(radius+ wheelToCenter)+1)

	while(theta < thetaGoal -tol or theta > thetaGoal+tol):
		spinWheels(leftmotor, rightmotor, poleRate)
		getOdom()
		print "theta: %f" % (theta)
		
		
		f.write(str.format("{0:.3f}", x))
		f.write(",")
		f.write(str.format("{0:.3f}", y))
		f.write("\n")





#Odometry Callback function.
def read_odometry(msg):
    pass  # Delete this 'pass' once implemented
  


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        pass  # Delete this 'pass' once implemented



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
	pass # Delete this 'pass' once implemented
def mapCallback(data):
    global xMapOffset
    global yMapOffset
    global mapRez
    global rvizmap
    global mapWidth 
    global mapHieght
    xMapOffset = data.info.origin.position.x
    yMapOffset = data.info.origin.position.y
    mapRez = data.info.resolution
    rvizmap = data.data
    mapWidth = data.info.width
    mapHieght = data.info.height

def OdomCallBack(data):
	px = data.pose.pose.position.x
	py = data.pose.pose.position.y
	quat = data.pose.pose.orientation
	q=[quat.x, quat.y, quat.z, quat.w]
	roll, pitch, yaw = euler_from_quaternion(q)

	global x
	global y
	global theta

	x = px
	y = py
	theta = yaw
def getOdom():
	sub = rospy.Subscriber("odom",Odometry, OdomCallBack)

def checkBumper():

	bumper_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, BumperEventCallback)


def BumperEventCallback(data):
	global bumper
	if(data.state == BumperEvent.RELEASED):
		bumper =0
	else:
		bumper =1
def publishGrid (nodeList):
	pub = rospy.Publisher('/path', GridCells, queue_size=1)
	gridCell = GridCells()
        gridCell.cell_width = mapRez
        gridCell.cell_height = mapRez
        gridCell.header.frame_id = "map"
        gridCell.cells = nodeList
	pub.publish(gridCell)
         
def publishTwist (u,w):
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
	twist = Twist()
	twist.linear.x =u;##
	twist.linear.y =0;##
	twist.linear.z =0;##
	twist.angular.x = 0;##
	twist.angular.y = 0;##
	twist.angular.z = w;##
	pub.publish(twist)


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')


    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global pub
    global pose
    global odom_tf
    global odom_list
    global bumper
    bumper=0
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1) # Publisher for commanding robot motion
    sub = rospy.Subscriber('odom', Odometry, OdomCallBack, queue_size=1) # Callback function to read in robot Odometry messages
    
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, BumperEventCallback, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    map_sub = rospy.Subscriber("/map", OccupancyGrid, mapCallback)
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    #testmap = [[100,100,100,100,100],[100,100,0,100,100],[100,100,0,100,100],[0,0,0,100,0],[100,100,100,100,0]]
    #print printPath(generateWaypoints(aStar(testmap, Node(4,0,46), Node(2,0,0))))
    time.sleep(.5)
    #print listToMapXY(rvizmap, 37, 37)
    #printPath(generateWaypoints(
    grid =aStar(listToMapXY(rvizmap, 37, 37), Node(1,1,46), Node(26,29,0))#))
    printPath(grid)
    grid = gridNodesToMeter(grid,xMapOffset, yMapOffset, .2)
    while 1:
        publishGrid (grid)
        time.sleep(.5)
    
    #f=open('trajectory', 'w')
    print "Starting Lab 2"
    #spinWheels(-.2, .2, 2)
    #driveStraight(.4, 1)
    #rotate(3.14)
    #driveArc(.3, .3, 3.14/2)
    #executeTrajectory()    
    # Make the robot do stuff...

    print "Lab 2 complete!"
