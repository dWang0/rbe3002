#!/usr/bin/env python
import roslib
import rospy, tf
import math
import time 
from Astar import *

from Map import *
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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
from std_msgs.msg import Header
from std_msgs.msg import Float64

#this function turns the grid value into a pose value by placing the next node in a 
#list which we use later in the driving function.
def gridNodesToPose(nodeList, xOffset, yOffset, rez):
    workingList =[]
    for node in nodeList:
        workingList.append(Node((node.x*rez+xOffset+(rez/2)), (node.y*rez+yOffset+(rez/2))))
    return workingList


#turns the pose into a grid cell location
def poseTogridNode(x, y, xOffset, yOffset, rez):
    return Node(int((x-xOffset)/rez), int((y-yOffset)/rez))



#this checks to see what known nodes have neighbors
#that are unknown. Then the goal is set to 
def isNear(currentNode):
    print "In nodes"
    global exploreList
    a = []
    rotate(333)
    
    while (safeOff is True) and (not exploreList):
        a = BFS(currentNode)
        print "in BFS while"
    
    print "after while loop"
    print a     
    if len(a) is 0:
        rospy.sleep(100000)
    else:
        endNode = a[0]
        publishStart(currentNode, '/start')
        publishEnd(endNode, '/frontierNode')
    
 
#called publisher that publishes the goal point
def publishEnd (frontierNode, name):
    pub = rospy.Publisher(name, PoseWithCovarianceStamped, queue_size=1)
    poseStart = PoseWithCovarianceStamped()
    poseStart.pose.pose.position.x = frontierNode.x
    poseStart.pose.pose.position.y = frontierNode.x
    poseStart.header.frame_id = "map"
    pub.publish(poseStart)
	
	
#called publisher that publishes the start point
def publishStart (currentNode, name):
	pub = rospy.Publisher(name, PoseStamped, queue_size=1)
	poseEnd = PoseStamped()
        poseEnd.pose.position.x = currentNode.x
        poseEnd.pose.position.y = currentNode.x
        poseEnd.header.frame_id = "map"
	pub.publish(poseEnd)
   

    
#BFS search    
def BFS(start):
    global safeOff
    visitedNodes = []
    queue = ([start])
    nodeUnknown = []
    while len(queue) > 0:
        anode = queue.pop()
        if anode in visitedNodes:
            continue
        
        visitedNodes.append(start)
        if (rvizmap[int(anode.x * anode.y)] is -1):
            nodeUnknown.append(anode)
            safeOff = False
            return nodeUnknown
        
        for nodes in anode.getBuds:
            if nodes not in visitedNodes:
                queue.appendleft(n)
    safeOff = False
    return nodeUnknown




#returns a list of waypoints for the current path    
def getWaypoints(nodeList):
    waypoints = []
    i = -1
    
    for node in nodeList:
        i += 1
        if(i==0):
            waypoints.append(node)
        elif(i==len(nodeList)-1):
            waypoints.append(node)
        else:
            dxA =nodeList[i-1].x-node.x
            dyA =nodeList[i-1].y-node.y
            dxB =node.x-nodeList[i+1].x
            dyB =node.y-nodeList[i+1].y
            if(not(dxA == dxB and dyA == dyB)):
                waypoints.append(node)
    return waypoints

#prints the path
def printPath(path):
    step = 1
    for n in path:
        print"step %f x: %f y: %f t: %f" % (step, n.x, n.y, 0 )#n.theta)
        step = step + 1

#prints the path to rviz
def rvizToNodeList(rmap, width):
    alist = []
    for i in range(len(rmap)):        
        if(rmap[i] == 0):
            newNode = Node(i%width, int(i/width))
            alist.append(newNode)
    return alist
    
#prints the walls to rviz    
def rvizToWallList(rmap, width):
    alist = []
    for i in range(len(rmap)):        
        if(rmap[i] == 100):
            newNode = Node(i%width, int(i/width))
            alist.append(newNode)
    return alist

#This is the driving calculation function. It drives to the next waypoint in the list of
#waypoints. After driving one grid cell length (.3m) it will recalculate
#to check for obstacles or more efficient paths.  
def driveToPoint(wayPoints, end):
    currentPos = wayPoints[0]
    global grid
    
    while((not currentPos == end) and (not len(wayPoints) <= 1)):
        currentPos = wayPoints.pop(0)
        posX, posY, posZ = getMapPose()
        yaw = getMapRot()
        print yaw
        
        theta = ((((currentPos.angleOfLine(wayPoints[0]) - yaw) + math.pi) %(2*math.pi)) - math.pi)
        distance = math.sqrt((currentPos.x - wayPoints[0].x)**2 + (currentPos.y - wayPoints[0].y)**2)
        rotate(theta)
        
        if(int(distance/.3)):
            driveStraight(.3,.2)
        else:
            driveStraight(distance, .2)
        
        while(mapBeingMade):
            time.sleep(.01)
       
        posX, posY, posZ = getMapPose()
        yaw = getMapRot()
        path = Astar().Astar(rmap, poseTogridNode(posX, posY, xMapOffset, yMapOffset, rmap.rez), poseTogridNode(goalx, goaly, xMapOffset, yMapOffset, rmap.rez))
        way = getWaypoints(path)
        grid= gridNodesToPose(path,xMapOffset, yMapOffset, rmap.rez)
        wayPoints = gridNodesToPose(way,xMapOffset, yMapOffset, rmap.rez)
        print len(wayPoints)
        
    driveStraight(.3,.2)

#This drives the robots forward in a straight line
def driveStraight(speed, distance):
	poleRate = .1
	tol = .1
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
		print "x: %f" % (x)
		print "y: %f" % (y)
		time.sleep(poleRate)
	publishTwist(0,0)

#This rotates the robot by the given angle
def rotate(angle):
    print "rotate"
    poleRate = .1
    tol = .05

    thetaStart = theta

    print "theta: %f" % (theta)
    
    thetaGoal = (((thetaStart+angle)+3.14)%(2*3.14))-3.14
    print "thetaGoal: %f" % (thetaGoal)

    while(theta < thetaGoal -tol or theta > thetaGoal+tol):
        print "current: ", theta
        print "desired: ", thetaGoal
        publishTwist(0, -.1)
        print "theta: %f" % (theta)
        time.sleep(poleRate)

    publishTwist(0,0)

       
       
'''
SUB CALLBACKS
'''
#map callback
def getMapPose():
    return listener.lookupTransform('/map', 'base_footprint', rospy.Time(0))[0]

#map rotation callback
def getMapRot():
    (trans,rot) = listener.lookupTransform('/map', 'base_footprint', rospy.Time(0))
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot) 
    return yaw

#odom callback   
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
    
    
    
#map callback 
def mapCallback(data):
    global xMapOffset
    global yMapOffset
    global mapRez
    global rvizmap
    global mapWidth 
    global mapHeight
    global rmap
    global floor
    global walls
    global mapBeingMade
    
    mapBeingMade = 1
    xMapOffset = data.info.origin.position.x
    yMapOffset = data.info.origin.position.y
    mapRez = data.info.resolution
    rvizmap = data.data
    mapWidth = data.info.width
    mapHieght = data.info.height
    rmap = Map(rvizmap, mapWidth, mapRez)
    rmap.compress(3)
    rmap.expandWalls(1)
    rmap.compress(2)
    print "map Made"
    walls = gridNodesToPose(rvizToWallList(rmap.list, rmap.width),xMapOffset, yMapOffset, rmap.rez)
    floor = gridNodesToPose(rvizToNodeList(rmap.list, rmap.width),xMapOffset, yMapOffset, rmap.rez)
    
    
    mapBeingMade = 0
    
    
    
#goal callback
def goalCallback(data):
    global goalx
    global goaly
    global goalTheta
    global grid
    global way
    
    goalx = data.pose.position.x
    goaly = data.pose.position.y
    goalTheta = data.pose.orientation.z
    posX, posY, posZ = getMapPose()
    yaw = getMapRot()
    startNode = poseTogridNode(posX, posY, xMapOffset, yMapOffset, rmap.rez)
    goalNode = poseTogridNode(goalx, goaly, xMapOffset, yMapOffset, rmap.rez)
    path = Astar().Astar(rmap, poseTogridNode(posX, posY, xMapOffset, yMapOffset, rmap.rez), poseTogridNode(goalx, goaly, xMapOffset, yMapOffset, rmap.rez))
    waypoints = getWaypoints(path)
    grid= gridNodesToPose(path,xMapOffset, yMapOffset, rmap.rez)
    way = gridNodesToPose(waypoints,xMapOffset, yMapOffset, rmap.rez)
    publishGrid (grid ,'/path')
    publishGrid (way ,'/way')
    if(not(len(way)==0)):
        driveToPoint(way, way[len(way)-1])

#start callback
def startCallback(data):
    global startx
    global starty
    global startTheta
    global startNode
    startx = data.pose.pose.position.x
    starty = data.pose.pose.position.y
    startTheta = data.pose.pose.orientation.z
    startNode = poseTogridNode(startx, starty, xMapOffset, yMapOffset, rmap.rez)




'''
PUBISHERS
'''
#grid publisher 
def publishGrid (nodeList, where):
	pub = rospy.Publisher(where, GridCells, queue_size=1)
	gridCell = GridCells()
        gridCell.cell_width = rmap.rez
        gridCell.cell_height = rmap.rez
        gridCell.header.frame_id = "map"
        gridCell.cells = nodeList
	pub.publish(gridCell)

#twist publisher	
def publishTwist (u,w):
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
	twist = Twist()
	twist.linear.x =u;##
	twist.linear.y =0;##
	twist.linear.z =0;##
	twist.angular.x = 0;##
	twist.angular.y = 0;##
	twist.angular.z = w;##
	pub.publish(twist)


#this wil run the robot in autonomous mode to scan the area
def totheFrontiersAndBeyond():
    global grid
    global way
    global rmap
    global floor
    global walls
    global exploreList
    global listener
    global safeOff
    global x , y
    safeOff = True
    exploreList = []
    walls = []
    floor = [] 
    grid = []
    way = [] 
    rmap = Map([], 0, 1)
    
    listener = tf.TransformListener()
    map_sub = rospy.Subscriber("/map", OccupancyGrid, mapCallback)
    goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalCallback)
    start_sub =rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, startCallback)
    odom_sub = rospy.Subscriber("odom",Odometry, OdomCallBack)
    time.sleep(.5)
    currentLoc = Node(x,y)
    isNear(currentLoc)
   
    while True:
        
        publishGrid (walls ,'/walls')
        publishGrid (floor ,'/floor')
        publishGrid (grid ,'/path')
        publishGrid (way ,'/way')
        time.sleep(.5)
    


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('lab4')
    totheFrontiersAndBeyond()


    
    
    
