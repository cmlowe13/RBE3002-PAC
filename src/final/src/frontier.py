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
	global nodes,frontPub, frontier
	gridCells = GridCells()
	gridCells.cell_width = 1
	gridCells.cell_height = 1
	gridCells.header.frame_id = 'map'
	gridCells.cells = [];
	frontier = [];
	for node in nodes:
		if node.type == 2:
			neighbors = getNeighbors(nodes,node)
			for n in neighbors:
				if n.type == 0:
					p = Point()
					p.x = node.x
					p.y = node.y
					p.z = 0
					gridCells.cells.append(p)
					frontier.append(node)
	frontPub.publish(gridCells)

class Blob:
	def __init__(self):
		self.size = 0
		self.nodes = []

	def add(self,node):
		self.size = self.size + 1
		self.nodes.append(node)

	def addAll(self,nodes):
		for n in nodes:
			self.add(n)

	def getCentroid(self):
		sumX = numpy.sum([n.x for n in self.nodes])
		sumY = numpy.sum([n.y for n in self.nodes])
		return sumX/self.size,sumY/self.size

def blobFront():
	global frontier,blobs
	toSearch = frontier
	while len(toSearch):
		node = toSearch.pop()
		blobList = getConnected(toSearch,node)
		toSearch = [x for x in toSearch if x not in blobList]
		blob = Blob()
		blob.addAll(blobList)
		blobs.append(blob)
	print blobs


def getConnected(list,node):
	toReturn = []
	for n in getNeighbors(list,node):
		list.remove(n)
		toReturn.append(getConnected(list,n))
	return toReturn

def getNeighbors(list,node):
	neighbors = []
	for n in list:
		if n.x == node.x + 1 and n.y == node.y:
			neighbors.append(n)
		if n.x == node.x - 1 and n.y == node.y:
			neighbors.append(n)
		if n.y == node.y + 1 and n.x == node.x:
			neighbors.append(n)
		if n.y == node.y - 1 and n.x == node.x:
			neighbors.append(n)
	return neighbors


def mapCallback(occupancy):
	global grid
	grid = occupancy
	buildNodes()
	getFront()
	

if __name__=='__main__':
	rospy.init_node('frontier_node')

	global grid,nodes,frontPub,frontier
	grid = OccupancyGrid()
	rospy.Subscriber('/map',OccupancyGrid, mapCallback)
	frontPub = rospy.Publisher('front_color',GridCells, queue_size = 1)

	rospy.spin()