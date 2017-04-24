#!/usr/bin/env python

import rospy, numpy, math
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells, Path
from geometry_msgs.msg import Point,PointStamped, PoseStamped

class Node:

	def __init__(self,x,y,type):
		self.x = x
		self.y = y
		self.gCost = 0
		self.type = type #0 if open, 1 if closed, 2 if unknown

	def setFCost(self,goal):
		self.fCost = self.gCost + (goal.x-self.x)**2 + (goal.y-self.y)**2
		return self.fCost

	def setCameFrom(self,x):
		self.cameFrom = x

def buildNodes():
	global grid,nodes
	nodes = []
	xi = 0
	yi = 0
	for cell in grid.data:
		if cell == 100:
			nodes.append(Node(xi,yi,0))
		elif cell== 0:
			nodes.append(Node(xi,yi,1))
		elif cell == -1:
			nodes.append(Node(xi,yi,2))
		if xi == grid.info.width-1:
			xi = 0
			yi = yi + 1
		else:
			xi = xi + 1

def getFront():
	global nodes,frontPub
	gridCells = GridCells()
	gridCells.cell_width = 1
	gridCells.cell_height = 1
	gridCells.header.frame_id = 'map'
	gridCells.cells = [];
	for x in range(50):
		if 1 == 1:
			p = Point()
			p.x = x
			p.y = 0
			p.z = 0
			gridCells.cells.append(p)
	frontPub.publish(gridCells)

def mapCallback(occupancy):
	global grid
	grid = occupancy
	buildNodes()
	getFront()
	

if __name__=='__main__':
	rospy.init_node('frontier_node')

	global grid,nodes,frontPub
	grid = OccupancyGrid()
	rospy.Subscriber('/map',OccupancyGrid, mapCallback)
	frontPub = rospy.Publisher('front_color',GridCells, queue_size = 1)

	rospy.spin()