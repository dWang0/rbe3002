#!/usr/bin/env python

import sys
import rospy
from lab3.srv import *
from geometry_msgs import * 
from nav_msgs import *

def calc_a_star_client(startPose, endPose):
    rospy.wait_for_service('calc_a_star')
    try:
        calc_a_star = rospy.ServiceProxy('calc_a_star', calc_a_star)
        resp1 = calc_a_star(startPose, endPose)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = Pose(sys.argv[1])
        y = Pose(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "here is your pose:"
    #print "Requesting from %(Pose) to %(Pose)"%(x, y)
    #^ How to print Poses as variables to replace in line
    #print "%s + %s = %s"%(x, y, calc_a_star_client(x, y))
