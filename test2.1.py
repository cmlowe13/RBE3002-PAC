#!/usr/bin/env python
#imports for lab 2
import rospy, tf, math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


#drive to a goal subscribed as /turtleGoal
def navToPose(goal):
	global pose
	print "goal receive"
	diff_x = goal.pose.position.x - pose.pose.position.x
	diff_y = goal.pose.position.y - pose.pose.position.y
	print "y: ", diff_y
	print "x: ", diff_x
	rotate(math.atan2(diff_y, diff_x))
	driveStraight(.1, math.sqrt((diff_x**2) + (diff_y**2)))
	rotate(math.degrees(goal.pose.orientation.z - pose.pose.orientation.z))

def publishTwist(lin, ang):
	global pub
	msg = Twist()
	msg.linear.x = lin 
	msg.angular.z = ang
	pub.publish(msg)

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	driveStraight(.3, .6)# drives straight at a speed of .3 for 60cm
	rotate(-90) #turns right 90 degrees
	driveStraight(.3, .45) # drives straght at a speed of .3 for 45cm
	rotate(135)# turns left 135 degreees
	print "Trajectory Complete"

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):

	lin_vel = (u1 + u2)/2# the linear velocity of the robot is the average of the wheel speeds
	ang_vel = (u1 - u2)/0.23# the angular velocity is equal to the difference of the wheel speeds divided by the distance between the wheels

	now = rospy.Time.now().secs # gets current time in seconds and stores it as varialbe now

	while(rospy.Time.now().secs - now <= time and not rospy.is_shutdown()): #while current time is less than input time
		publishTwist(lin_vel, ang_vel); # pulishes the calculated linear and angular velocities
	publishTwist(0, 0) # stop robot after time has elapsed

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	global pose# glbobal pose
	ix = pose.pose.position.x ##stores initial x,y of the robot
	iy = pose.pose.position.y 
	reachedGoal = False #has not yet reached goal 

	while(not reachedGoal and not rospy.is_shutdown()):
		d_x = pose.pose.position.x - ix	##updates difference between initial x,y
		d_y = pose.pose.position.y - iy
		c_dis = math.sqrt((d_x * d_x) + (d_y * d_y)) # calculates distance driven using distance formula from initial to final position
		if(c_dis >= distance): # if the robot has driven the input distance, then it has reached its goal
			reachedGoal = True 
		publishTwist(speed, 0) # else publishes a linear velocity to  twist equivalent to the input
	print "drvestraight done"
	publishTwist(0, 0) #once the while is exited, stop the robot

def transformAngle(angle): ### ignore this, my attempt to take -180-180 and turn it into 0-360
	if(angle >= 0):
		return angle
	else:
		return angle + 360

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):

	global pose

	if(angle > 180 or angle < -180): # if angle is out of range, prints invalid angle
		print "invalid angle"

   	done = False 
   	dest = angle + math.degrees(pose.pose.orientation.z) # set destination angle equal to the angle + the degrees equivalent of your current angle
   	error = dest - math.degrees(pose.pose.orientation.z) # error = destination angle - current angle 
	direction = error/abs(error) # becomes either -1 or 1 based upon the direction the error is in
	angVel = .7 # constant angular vel

	while(not done and not rospy.is_shutdown()):
		direction = error/abs(error) ##updates direction and error at the beginnning of the loop
		error = dest - math.degrees(pose.pose.orientation.z)
		if(abs(error) <= 2): # 4 degree total margin of error, 2 on either side of desired angle
			done = True
		publishTwist(0, angVel*direction) # angvel will either be negative or positive based on direction 
		print "error: ", error
		print "theta: ", math.degrees(pose.pose.orientation.z)
	publishTwist(0,0)
	print "rotate done"

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	pass  # Delete this 'pass' once implemented





#Bumper Event Callback function
def readBumper(msg):

	if (msg.state == 1):
		print "Bumper Pressed"
		executeTrajectory()

#Timer Event Callback function 
#updates the pose and odometry of the robot
def timerCallback(event):

	global pose
	global xPosition
	global yPosition
	global theta

	odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
	(position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
	pose.pose.position.x = position[0]
	pose.pose.position.y = position[1]
	xPosition = position[0]
	yPosition = position[1]

	odomW = orientation
	q = [odomW[0], odomW[1], odomW[2], odomW[3]]
	roll, pitch, yaw = euler_from_quaternion(q)

   	pose.pose.orientation.z = yaw
   	theta = math.degrees(yaw)

# This is the program's main function
if __name__ == '__main__':

	rospy.init_node('coshea_Lab_2_node') #initialized ROS nodee

	# Globals to define publishers and pose/odometry
	global pub
	global pose
	global odom_tf
	global odom_list
	
	#publishers and subscribers for lab2
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
	bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
	rviz_sub = rospy.Subscriber('/turtleGoal', PoseStamped, navToPose) # Callback function for a 2-D navigation command from rviz
	# Use this object to get the robot's Odometry 
	odom_list = tf.TransformListener()
	pose = PoseStamped()# pose stamped for running on the turtlebot

	print "Starting Lab 2" #BEGINNING OF LAB2 CODE TO BE EXECUTED

	rospy.Timer(rospy.Duration(.01), timerCallback) #updates pose and odometry on set time interval, similar to an interrupt in c
	notDone = True
	while(notDone):
		a = 0


	print "Lab 2 complete!" #END OF LAB2 CODE TO BE EXECUTED

