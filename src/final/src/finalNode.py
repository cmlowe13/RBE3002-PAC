#!/usr/bin/env python

import rospy, numpy, math
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells, Path
from geometry_msgs.msg import Point,PointStamped, PoseStamped, PoseWithCovarianceStamped

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


def reconstruct_path(node):
	path = [node]
	current = node
	while hasattr(current,'cameFrom'):
		path.append(current)
		current = current.cameFrom
	return path

def astar(start, end):
	global nodes,frontPub, closedPub
	print(start.x,start.y,end.x,end.y)
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
		gridCells = GridCells()
		gridCells.cell_width = 1
		gridCells.cell_height = 1
		gridCells.header.frame_id = 'map'
		gridCells.cells = [];
		for node in closedSet:
			p = Point()
			p.x = node.x
			p.y = node.y
			p.z = 0
			gridCells.cells.append(p)
		closedPub.publish(gridCells)

		lowest = 99999
		for n in openSet:
			if(n.setFCost(end)<=lowest):
				print(n.x,n.y,n.gCost)
				lowest = n.fCost
				current = n
		print("loop")
		openSet = [n for n in openSet if not(n.x == current.x and n.y == current.y)]
		if current.x == end.x and current.y == end.y:
			return reconstruct_path(current)
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
			if [item for item in closedSet if (item.x == n.x and item.y == n.y)]:
				1+1
			else:
				n.setCameFrom(current)
				n.gCost = current.gCost + 1
				n.setFCost(end)		
				openSet.append(n)
		closedSet.append(current)

	return 0

def genWaypoints (gen_Path_rev):
	
	gen_pts = []
	w_ori = []
	gen_Path = list(reversed(gen_Path_rev))

	prev_pts = gen_Path[0]
	tmp_ctr = 0
	for a, point in enumerate(gen_Path):
		if a ==len(gen_Path) - 1:
			gen_pts.append(gen_Path[-1])

			q_t = quaternion_from_euler(0,0,0)
			quaternion_h = Quaternion(*q_t)

			w_ori.append(quaternion_h)
		elif tmp_ctr == 3:
			tmp_ctr = 0
			gen_pts.append(point)

			dx = point.x - prev_pts.x
			dy = point.y - prev_pts.y
			heading = (dx, dy)
			th_heading = math.atan2(heading[1], heading[0])
			q_t = quaternion_from_euler(0,0, th_heading)
			quaternion_h = Quaternion(*q_t)

			w_ori.append(quaternion_h)
			prev_pts = point
		else:
			tmp_ctr += 1

	path = Path()
	for a in range(len(gen_pts)):
		path.poses.append(PoseStamped(Header(), Pose(gen_pts[a], w_ori[a])))
	return path

def publishTwist(lin_vel, ang_vel):
	global prev_twist
	global nav_pub

   twist_msg = Twist();                #Create Twist Message
    if lin_vel == 0 and ang_vel == 0:
        twist_msg.linear.x = (prev_twist.linear.x)/3
        twist_msg.angular.z = (prev_twist.angular.z)/3
        while twist_msg.linear.x > 0.05 and twist_msg.angular.z > 0.05:
            twist_msg.linear.x = (prev_twist.linear.x)/3
            twist_msg.angular.z = (prev_twist.angular.z)/3
            prev_twist = twist_msg
            nav_pub.publish(twist_msg)              #Send Message
            rospy.sleep(rospy.Duration(0.2, 0))
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        nav_pub.publish(twist_msg)
        prev_twist.linear.x = 0
        prev_twist.angular.z = 0
    else:
        twist_msg.linear.x = (2*lin_vel + prev_twist.linear.x)/3
        twist_msg.angular.z = (2*ang_vel + prev_twist.angular.z)/3
        prev_twist = twist_msg
        nav_pub.publish(twist_msg)    

def navToPose(goal):
    global pose

    x0 = pose.pose.position.x        #Set origin
    y0 = pose.pose.position.y
    q0 = (pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w)
    x1 = goal.pose.position.x
    y1 = goal.pose.position.y
    q1 = (goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w)
    theta_tup_0 = euler_from_quaternion(q0)
    theta0 = theta_tup_0[2]
    theta_tup_1 = euler_from_quaternion(q1)
    theta2 = theta_tup_1[2]

    dx = x2 - x0
    dy = y2 - y0
    theta1 = math.atan2(dy, dx)

    dtheta0 = theta1 - theta0
    dtheta1 = theta2 - theta1
    distance = math.sqrt(dx**2 + dy**2)

    rotate(dtheta0)
    driveStraight(0.1, distance)
    rotate(dtheta1)

def rotate(angle):
    global odom_list
    global pose

    # This node was created using Coordinate system transforms and numpy arrays.
    # The goal is measured in the turtlebot's frame, transformed to the odom.frame 
    transformer = tf.TransformerROS()
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],     #Create goal rotation
                            [math.sin(angle),  math.cos(angle), 0],
                            [0,                0,          	1]])

    # Get transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

    # Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                          [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                          [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                          [0,             0,             0,             1]])

   # Continues creating and matching coordinate transforms.
    done = False
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .2
        if (within_tolerance.all()):
            publishTwist(0, 0)
            done = True
        else:
            if (angle > 0):
                publishTwist(0, 0.5)
            else:
                publishTwist(0, -0.5)

def driveStraight(speed, distance):
    global pose

    x0 = pose.pose.position.x   #Set origin
    y0 = pose.pose.position.y

    done = False
    while (not done and not rospy.is_shutdown()):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y
        d = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)      #Distance formula
        if (d >= distance):
            publishTwist(0, 0)
            done = True
        else:
            publishTwist(speed, 0)

def readOdom(msg):
    global pose
    global odom_tf

    pose = msg.pose
    geo_quat = pose.pose.orientation
    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
                        (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
                        rospy.Time.now(),
                        "base_footprint","odom")

def mapCallback(occupancy):
	global grid
	grid = occupancy
	buildNodes()

def setPose(est_pose):
	global pose
	pose = est_pose

if __name__=='__main__':
	rospy.init_node('main_node')

	global grid,nodes,frontPub,frontier,blobs,pose
	grid = OccupancyGrid()
	rospy.Subscriber('/buffermap',OccupancyGrid, mapCallback)
	rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,setPose)
	frontPub = rospy.Publisher('front_color',GridCells, queue_size = 1)

	state = 0 # 0: spin, 1: blob frontier, 2: nav to blob,  3: done
	i_waypoints = 0

	while not rospy.is_shutdown():
		if state == 0:
			if not spinRobot():
				state = 1
		elif state == 1:
			if(grid.data)
				buildNodes()
				getFront()
				if len(frontier) == 0:
					state = 3
					continue
				blobFront()
				state = 2
		elif state == 2:
			heavyBlob = None
			heaviest = 0
			for blob in blobs:
				if(blob.size > heaviest):
					heavyBlob = blob
					heaviest = blob.size
			path = astar(Node(pose.x,pose.y,0),Node(heavyBlob.getCentroid()[0],heavyBlob.getCentroid()[1]))
			wpoints = genWaypoints(path)
			if i_waypoints > len(wpoints)-1:
				i_waypoints = 0
			else:
				navToPose(wpoints[i_waypoints])
				i_waypoints = i_waypoints + 1
			state = 1
		elif state == 3:
			print("done")
			break

