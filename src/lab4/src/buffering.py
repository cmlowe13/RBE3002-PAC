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

def buildNodes():
	global grid,nodes
	nodes = []
	xi = 0
	yi = 0
	for cell in grid.data:
		if cell == 100:
			nodes.append(Node(xi,yi))
		if xi == grid.info.width-1:
			xi = 0
			yi = yi + 1
		else:
			xi = xi + 1

def buffering():
	global nodes
	oldnodes = nodes
	for current in nodes:
		for n in oldnodes:
			if not (n.x == current.x + 1 and n.y == current.y):
				nodes.append(Node(current.x +1, current.y))
			if not (n.x == current.x - 1 and n.y == current.y):
				nodes.append(Node(current.x +1, current.y))
			if not (n.y == current.y + 1 and n.x == current.x):
				nodes.append(Node(current.x +1, current.y))
			if not (n.y == current.y - 1 and n.x == current.x):
				nodes.append(Node(current.x +1, current.y))

def colored():
	global nodes
	gridCells = GridCells()
	for node in nodes:
		p = Point()
		p.x = node.x
		p.y = node.y
		p.z = 0
		gridCells.cells.append(p)
	bufferPub.publish(gridCells)

def mapCallback(occupancy):
	global grid
	grid = occupancy
	

if __name__=='__main__':
	rospy.init_node('buffer_node')

	global grid,nodes,bufferPub
	grid = OccupancyGrid()
	rospy.Subscriber('/map',OccupancyGrid, mapCallback)
	bufferPub = rospy.Publisher('/buffer_color',GridCells, queue_size = 1)

	
	rospy.sleep(.5)
	print("building")
	buildNodes()
	print("buffering")
	buffering()
	print("coloring")
	colored()
	print("done")
