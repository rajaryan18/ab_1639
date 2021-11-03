#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

pose = [0.8, -1.3609, 1.570159]
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
	regions = {
		'bright': min(min(msg.ranges[0:119]), range_max),
		'fright': min(min(msg.ranges[120:239]), range_max),
		'rfront': min(min(msg.ranges[240:359]), range_max),
		'lfront': min(min(msg.ranges[360:479]), range_max),
		'fleft': min(min(msg.ranges[480:599]), range_max),
		'bleft': min(min(msg.ranges[600:719]), range_max),
	}


def forward(pub, regionOne, distOne, regionTwo, distTwo, last):
	global regions
	velocity_msg = Twist()
	rate = rospy.Rate(20)
	if last == True:
		while regions[regionOne] < distOne or regions[regionTwo] > distTwo:
			velocity_msg.linear.x = 0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.linear.x = 0
		pub.publish(velocity_msg)
	else:
		while regions[regionOne] < distOne or regions[regionTwo] < distTwo:
			velocity_msg.linear.x = 0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.linear.x = 0
		pub.publish(velocity_msg)


def moveExtra(pub, direction, dist):
	global pose
	velocity_msg = Twist()
	rate = rospy.Rate(20)
	if direction == 'left':
		x = pose[0]
		while pose[0] > (x-dist):
			velocity_msg.linear.x = 0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.linear.x = 0
		pub.publish(velocity_msg)
	elif direction == 'right':
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
		while pose[1] < (y+dist):
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


def rotate(pub, direction, angle):
	global pose
	velocity_msg = Twist()
	rate = rospy.Rate(20)
	if direction == 'left':
		ch = 1
		while pose[2] < angle:
			if angle == 3.14 and pose[2] < 0:
				break
			velocity_msg.angular.z = 0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.angular.z = 0
		pub.publish(velocity_msg)
		if pose[2] != angle:
			while pose[2] < -2 or pose[2] > angle:
				velocity_msg.angular.z = -0.6
				pub.publish(velocity_msg)
				rate.sleep()
				ch = 0
			while pose[2] < angle and ch == 1:
				velocity_msg.angular.z = 0.6
				pub.publish(velocity_msg)
				rate.sleep()
				print("Controller message pushed at {}".format(rospy.get_time()))
			velocity_msg.angular.z = 0
			pub.publish(velocity_msg)
	else:
		while pose[2] < 0 and angle > 0:
			velocity_msg.angular.z = -0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		while pose[2] > angle:
			velocity_msg.angular.z = -0.6
			pub.publish(velocity_msg)
			rate.sleep()
			print("Controller message pushed at {}".format(rospy.get_time()))
		velocity_msg.angular.z = 0
		pub.publish(velocity_msg)
		if pose[2] != angle:
			ch = 1
			while pose[2] < angle:
				velocity_msg.angular.z = 0.6
				pub.publish(velocity_msg)
				rate.sleep()
				print("Controller message pushed at {}".format(rospy.get_time()))
				ch = 0
			while pose[2] > angle and ch == 1:
				velocity_msg.angular.z = -0.6
				pub.publish(velocity_msg)
				rate.sleep()
				print("Controller message pushed at {}".format(rospy.get_time()))
			velocity_msg.angular.z = 0
			pub.publish(velocity_msg)



def control_loop():
	global pose
	global regions
	PI = 3.1415926535897
	rospy.init_node('ebot_controller')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
	rospy.Subscriber('/odom', Odometry, odom_callback)
	rate = rospy.Rate(20) 
	velocity_msg = Twist()
	forward(pub, 'bleft', 2, 'lfront', 2, False)
	moveExtra(pub, 'up', 0.6)
	rotate(pub, 'right', 0)
	forward(pub, 'bright', 2, 'rfront', 2, False)
	moveExtra(pub, 'right', 0.65)
	rotate(pub, 'right', -1.59)
	forward(pub, 'bright', 2, 'rfront', 2, False)
	moveExtra(pub, 'down', 0.67)
	rotate(pub, 'right', 3.12)
	forward(pub, 'bright', 2, 'rfront', 2, False)
	moveExtra(pub, 'left', 0.65)
	rotate(pub, 'right', 1.57)
	forward(pub, 'bright', 2, 'rfront', 2, False)
	rate.sleep()


if __name__ == '__main__':
	try:
		control_loop()
	except rospy.ROSInterruptException:
		pass
