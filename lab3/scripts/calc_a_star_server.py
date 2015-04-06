#!/usr/bin/env python

from geometry_msgs import * 
from lab3.srv import *
from nav_msgs import *
import rospy

def handle_calc_a_star(apath):
    print "Calculating A* path..."
    #TODO: Function that calculated a*

def calc_a_star_server():
    rospy.init_node('calc_a_star_server')
    s = rospy.Service('calc_a_star', calc_a_star, handle_calc_a_star)
    print "Ready to calculate A* path..."
    rospy.spin()

if __name__ == "__main__":
    calc_a_star_server()
