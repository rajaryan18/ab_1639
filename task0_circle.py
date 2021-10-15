#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
class MoveTurtle:

	def __init__(self):

		self.curr_x = 0
		self.curr_y = 0
		self.curr_theta = 0
		self.curr_deg = 0

	def func_ros_sub_callback(self, pose_message):
		self.curr_x = pose_message.x
		self.curr_y = pose_message.y
		self.curr_theta = pose_message.theta
		if self.curr_theta < 0:
			self.curr_deg = 360 - abs(math.degrees(self.curr_theta))
		else:
			self.curr_deg = abs(math.degrees(self.curr_theta))
   
	def movCircle(self, ang_vel):
		obj_velocity_mssg = Twist()
		obj_pose_mssg = Pose()

		start_degree = 0
		current_degree = 0

		handle_pub_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		lin_vel = 0.8
		#ang_vel = 0.5
		max_degree = 360

		obj_velocity_mssg.linear.x = lin_vel
		obj_velocity_mssg.angular.z = ang_vel


		while round(current_degree) < round(max_degree):
			handle_pub_vel.publish(obj_velocity_mssg)

			current_degree = self.curr_deg - start_degree
			rospy.loginfo("Moving in a circle")

		obj_velocity_mssg.linear.x = 0
		obj_velocity_mssg.angular.z = 0
		handle_pub_vel.publish(obj_velocity_mssg)
        
        

def main():
	rospy.init_node('shapes_turtle', anonymous=True)         
	turtle = MoveTurtle()
	handle_sub_pose = rospy.Subscriber('/turtle1/pose', Pose, turtle.func_ros_sub_callback)

	turtle.movCircle(-0.8)
	
	turtle1 = MoveTurtle()
	handle_sub_pose1 = rospy.Subscriber('/turtle1/pose', Pose, turtle1.func_ros_sub_callback)

	turtle1.movCircle(0.8)
	print("Goal Reached")

main()
