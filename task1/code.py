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


def control_loop():
	global pose
	global regions
	PI = 3.1415926535897

	rospy.init_node('ebot_controller')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
	rospy.Subscriber('/odom', Odometry, odom_callback)
	rate = rospy.Rate(10) 

	velocity_msg = Twist()
	velocity_msg.linear.x = 0
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)
	#while not rospy.is_shutdown():
	while pose[2] < 3.1:
		velocity_msg.angular.z = 0.5
		pub.publish(velocity_msg)
		rate.sleep()
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)
	rate.sleep()
	while regions['bright'] < 2 or regions['rfront'] < 2:
		velocity_msg.linear.x = 0.5
		pub.publish(velocity_msg)
		rate.sleep()
	rate.sleep()

	x = pose[0]
	while pose[0] > (x-0.6):
		velocity_msg.linear.x = 0.5
		pub.publish(velocity_msg)
		rate.sleep()
	velocity_msg.linear.x = 0
	pub.publish(velocity_msg)
	rate.sleep()
	while pose[2] > PI/2:
		velocity_msg.angular.z = -0.5
		pub.publish(velocity_msg)
		rate.sleep()
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)

	while regions['bright'] < 2 or regions['rfront'] < 2:
		velocity_msg.linear.x = 0.7
		pub.publish(velocity_msg)
		rate.sleep()
	velocity_msg.linear.x = 0
	pub.publish(velocity_msg)

	y = pose[1]
	while pose[1] < (y+0.55):
		velocity_msg.linear.x = 0.7
		pub.publish(velocity_msg)
	velocity_msg.linear.x = 0
	pub.publish(velocity_msg)

	while pose[2] > 0.05:
		velocity_msg.angular.z = -0.5
		pub.publish(velocity_msg)
		rate.sleep()
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)
	rate.sleep()

	x = pose[0]
	while pose[0] < x+0.3 :
		velocity_msg.linear.x = 0.5
		pub.publish(velocity_msg)
		rate.sleep()

	while regions['bright'] < 1 or regions['fright'] > 1:
		velocity_msg.linear.x = 0.5
		pub.publish(velocity_msg)
		rate.sleep()
	velocity_msg.linear.x = 0
	pub.publish(velocity_msg)
	rate.sleep()

	while pose[2] > -1.53:
		velocity_msg.angular.z = -0.5
		pub.publish(velocity_msg)
		rate.sleep()
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)
	rate.sleep()

	while regions['bright'] < 1 or regions['bleft'] < 1:
		velocity_msg.linear.x = 0.7
		pub.publish(velocity_msg)
		rate.sleep()
	velocity_msg.linear.x = 0
	pub.publish(velocity_msg)
	rate.sleep()

	print("Controller message pushed at {}".format(rospy.get_time()))
	rate.sleep()




if __name__ == '__main__':
	try:
		control_loop()
	except rospy.ROSInterruptException:
		pass