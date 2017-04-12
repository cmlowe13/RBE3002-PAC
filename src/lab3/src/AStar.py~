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
		if cell == 0:
			nodes.append(Node(xi,yi))
		if xi == grid.info.width-1:
			xi = 0
			yi = yi + 1
		else:
			xi = xi + 1

	

def reconstruct_path(node):
	path = [node]
	current = node
	while hasattr(current,'cameFrom'):
		path.append(current)
		current = current.cameFrom
	return path

def astar(start, end):
	global nodes,frontPub
	closedSet = []
	openSet = []
	openSet.insert(0,start)
	current = start
	current.gCost = 0
	current.setFCost(end)

	while(openSet):
		gridCells = GridCells()
		gridCells.cell_width = 1
		gridCells.cell_height = 1
		gridCells.header.frame_id = 'map'
		gridCells.cells = [];
		for node in openSet:
			p = Point()
			p.x = node.x
			p.y = node.y
			p.z = 0
			gridCells.cells.append(p)
		frontPub.publish(gridCells)
		lowest = 99999
		for n in openSet:
			if(n.setFCost(end)<=lowest):
				lowest = n.fCost
				current = n
		
		if current.x == end.x and current.y == end.y:
			return reconstruct_path(current)

		openSet = [n for n in openSet if n.x != current.x and n.y != current.y]
		closedSet.append(current)
		neighbors = []
		for n in nodes:
			if n.x == current.x + 1 and n.y == current.y:
				neighbors.append(n)
			if n.x == current.x - 1 and n.y == current.y:
				neighbors.append(n)
			if n.y == current.y + 1 and n.x == current.x:
				neighbors.append(n)
			if n.y == current.y - 1 and n.x == current.x:
				neighbors.append(n)
		for n in neighbors:
			if [item for item in closedSet if item.x ==n.x and item.y == n.y]:
				continue
			tempCost = current.gCost + 1
			if not [item for item in openSet if item.x ==n.x and item.y == n.y]:
				openSet.append(n)
			elif tempCost >= n.gCost:
				continue
			n.setCameFrom(current)
			n.gCost = tempCost
			n.setFCost(end)

	return 0




def mapCallback(occupancy):
	global grid
	grid = occupancy

def clickCallback(click):
	global state,pathPub,start
	if state == 0:
		start = Node(int(click.point.x),int(click.point.y))
		state = 1
	elif state == 1:
		path = astar(start,Node(int(click.point.x),int(click.point.y)))
		gridCells = GridCells()
		gridCells.cell_width = 1
		gridCells.cell_height = 1
		gridCells.header.frame_id = 'map'
		gridCells.cells = [];
		if path == 0:
			print "No Path"
			raise SystemExit(0)
		for node in path:
			p = Point()
			p.x = node.x
			p.y = node.y
			p.z = 0
			gridCells.cells.append(p)
		pathPub.publish(gridCells)
		state = 2
	elif state == 2:
		gridCells = GridCells()
		gridCells.cell_width = 1
		gridCells.cell_height = 1
		gridCells.header.frame_id = 'map'
		gridCells.cells = [];
		pathPub.publish(gridCells)
		frontPub.publish(gridCells)
		state = 0



if __name__=='__main__':
	rospy.init_node('astar_node')

	global grid,nodes,pathPub,frontPub,state,start
	grid = OccupancyGrid()
	state = 0
	rospy.Subscriber('/map',OccupancyGrid, mapCallback)
	rospy.Subscriber('/clicked_point',PointStamped,clickCallback)
	pathPub = rospy.Publisher('map_color',GridCells, queue_size = 1)
	frontPub = rospy.Publisher('front_color',GridCells, queue_size = 1)
	

	rospy.sleep(.5)

	buildNodes()
	while not rospy.is_shutdown():
		1+1
