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
	global nodes, grid, buffer_pts
	buffer_pts = []
	ngrid = [0] * len(grid.data)
	oldnodes = nodes
	for x in range(grid.info.width):
		for y in range(grid.info.height):
			if (grid.data [x + y *grid.info.width]== -1):
				ngrid[x +y * grid.info.width] == -1
			if (grid.data [x +y *grid.info.width] == 100):
				for rx in range(2):
					for ry in range(2):
						ngrid = addAround(ngrid,x +rx, y +ry)
						ngrid = addAround(ngrid,x -rx, y +ry)
						ngrid = addAround(ngrid,x +rx, y -ry)
						ngrid = addAround(ngrid,x -rx, y -ry)
	return ngrid

def addAround(n, x, y):
	global ngrid
	global buffer_pts
	if (x < 0 or y < 0 or x >= grid.info.width or y >= grid.info.height):
		return n
	p = Point()
	p.x = x
	p.y = y
	buffer_pts.append(p)
	n[x + y * grid.info.width] = 100
	return n



def mapCallback(occupancy):
	global grid
	grid = occupancy
	

if __name__=='__main__':
	rospy.init_node('buffer_node')

	global grid,nodes,bufferPub
	grid = OccupancyGrid()
	rospy.Subscriber('/map',OccupancyGrid, mapCallback)
	bufferPub = rospy.Publisher('/buffermap',OccupancyGrid, queue_size = 1)

	
	rospy.sleep(.5)
	print("building")
	buildNodes()
	print("buffering")
	returngrid = OccupancyGrid()
	returngrid.info = grid.info
	returngrid.data = buffering()
	bufferPub.publish(returngrid)
	print("done")
