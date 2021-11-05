#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

pose = [0.8, -1.3609, 1.570159] #initial x, y and yaw values of the bot just after it spawns in the greenhouse
regions = {
	'bright': 1,
	'fright': 1,
	'rfront': 1,
	'lfront': 1,
	'fleft': 1,
	'bleft': 1
}


def odom_callback(data):
	global pose
	x = data.pose.pose.orientation.x;
	y = data.pose.pose.orientation.y;
	z = data.pose.pose.orientation.z;
	w = data.pose.pose.orientation.w;
	pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


def laser_callback(msg):
	range_max = 10.0
	global regions
	regions = {#setting the angles for the given parameteres for the bot to guide its way
		'bright': min(min(msg.ranges[0:119]), range_max),
		'fright': min(min(msg.ranges[120:239]), range_max),
		'rfront': min(min(msg.ranges[240:359]), range_max),
		'lfront': min(min(msg.ranges[360:479]), range_max),
		'fleft': min(min(msg.ranges[480:599]), range_max),
		'bleft': min(min(msg.ranges[600:719]), range_max),
	}


def forward(pub, regionOne, distOne, regionTwo, distTwo, last): #function to move the bot linearly
	global regions
	velocity_msg = Twist()
	rate = rospy.Rate(20)
	if last == True: 
		while regions[regionOne] < distOne or regions[regionTwo] > distTwo: #code for moving the bot from one line of troughs to the next one or to move the bot during the U turn 
			velocity_msg.linear.x = 0.6 #setting the linear speed
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.linear.x = 0 
		pub.publish(velocity_msg)
	else: # in the else block the code is written for moving the bot along the trough by checking the respective back or front parameters
		while regions[regionOne] < distOne or regions[regionTwo] < distTwo:
			velocity_msg.linear.x = 0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.linear.x = 0
		pub.publish(velocity_msg)


def moveExtra(pub, direction, dist): #function for slight extra movement of the bot in each turn to avoid any collision while turning 
	global pose
	velocity_msg = Twist()
	rate = rospy.Rate(20)
	if direction == 'left':   #while turning left 
		x = pose[0]
		while pose[0] > (x-dist): # while turing left we move linearly along the -ve x direction, we move a little extra
			velocity_msg.linear.x = 0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.linear.x = 0
		pub.publish(velocity_msg)
	elif direction == 'right': #simliar to the left turn we move a little extra in the +ve x direction while turning right, dist is the small extra distance we move
		x = pose[0]
		while pose[0] < (x+dist):
			velocity_msg.linear.x = 0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.linear.x = 0
		pub.publish(velocity_msg)
	elif direction == 'up': 
		y = pose[1]
		while pose[1] < (y+dist): # here we move slight extra distance in the positive y direction for a smooth turn without any collisions 
			velocity_msg.linear.x = 0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.linear.x = 0
		pub.publish(velocity_msg)
	else:
		y = pose[1]
		while pose[1] > (y-dist):
			velocity_msg.linear.x = 0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.linear.x = 0
		pub.publish(velocity_msg)


def rotate(pub, direction, angle): #function for the rotation of the bot, 'angle' is the desired angle/yaw parameter to which we want to rotate the bot
	global pose
	velocity_msg = Twist()
	rate = rospy.Rate(20)
	if direction == 'left':   
		ch = 1
		while pose[2] < angle:
			if angle == 3.14 and pose[2] < 0:
				break
			velocity_msg.angular.z = 0.5
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.angular.z = 0
		pub.publish(velocity_msg)
		if pose[2] != angle:
			while pose[2] < -2 or pose[2] > angle: #the bot will rotate as long as the desired angle of rotation os not attained
				velocity_msg.angular.z = -0.5
				pub.publish(velocity_msg)
				rate.sleep()
				ch = 0
			while pose[2] < angle and ch == 1:
				velocity_msg.angular.z = 0.5
				pub.publish(velocity_msg)
				rate.sleep()
				print("Controller message pushed at {}".format(rospy.get_time()))
			velocity_msg.angular.z = 0
			pub.publish(velocity_msg)
	else:
		while pose[2] > angle:
			velocity_msg.angular.z = -0.5
			pub.publish(velocity_msg)
			rate.sleep()print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.angular.z = 0
		pub.publish(velocity_msg)
		if pose[2] != angle:
			ch = 1
			while pose[2] < angle:
				velocity_msg.angular.z = 0.5
				pub.publish(velocity_msg)
				rate.sleep()
				print("Controller message pushed at {}".format(rospy.get_time()))
				ch = 0
			while pose[2] > angle and ch == 1:
				velocity_msg.angular.z = -0.5
				pub.publish(velocity_msg)
				rate.sleep()
				print("Controller message pushed at {}".format(rospy.get_time()))
			velocity_msg.angular.z = 0
			pub.publish(velocity_msg)



def control_loop(): #function for calling the functions written above 
	global pose
	global regions
	PI = 3.1415926535897
	rospy.init_node('ebot_controller')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
	rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback) 
	rospy.Subscriber('/odom', Odometry, odom_callback)
	rate = rospy.Rate(20) 
	velocity_msg = Twist()
	forward(pub, 'bleft', 2, 'lfront', 2, False) #calling the forward function with 'regionOne = 'bleft', 'distOne'=2 and so on'
	moveExtra(pub, 'up', 0.6) #moving slightly extra distance of 0.6 in the positive y direction 
	rotate(pub, 'left', 3.14) #rotating left to 3.14
	forward(pub, 'bleft', 2, 'lfront', 2, False)
	moveExtra(pub, 'left', 0.65)
	rotate(pub, 'left', -1.57)
	forward(pub, 'bleft', 2, 'lfront', 2, False)
	moveExtra(pub, 'down', 0.67)
	rotate(pub, 'left', -0.01)
	forward(pub, 'bleft', 2, 'lfront', 2, False)
	moveExtra(pub, 'right', 0.65)
	rotate(pub, 'left', 1.57)
	forward(pub, 'bleft', 2, 'lfront', 2, False)
	rate.sleep()


if __name__ == '__main__':
	try:
		control_loop()
	except rospy.ROSInterruptException:
		pass
