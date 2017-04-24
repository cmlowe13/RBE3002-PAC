#!/usr/bin/env python

import rospy, numpy, math
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells, Path
from geometry_msgs.msg import Point,PointStamped, PoseStamped


class Node:

	def __init__(self,x,y):
		self.x = x
		self.y = y
		self.gCost = 0

	def setFCost(self,goal):
		self.fCost = self.gCost + (goal.x-self.x)**2 + (goal.y-self.y)**2
		return self.fCost

	def setCameFrom(self,x):
		self.cameFrom = x


def drivePath(path):
	loc = path.cells[len(path.cells-1)]
	for n in reversed(path.cells):
		if()


if __name__=='__main__':
	rospy.init_node('drivePath_node')
	global pub
	rospy.Subscriber('/map_color',GridCells,drivePath,queue_size=1)
	pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist,None,queue_size=10)

	rospy.sleep(.5)

	while not rospy.is_shutdown():
		1+1
