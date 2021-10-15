#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist #for representation of angular and linear velocity
from turtlesim.msg import Pose # for representation of position and orientation
class MoveTurtle: 

	def __init__(self): #initialization function

		self.curr_x = 0
		self.curr_y = 0
		self.curr_theta = 0
		self.curr_deg = 0

	def func_ros_sub_callback(self, pose_message): 
		self.curr_x = pose_message.x # storing current positions/coordinates
		self.curr_y = pose_message.y 
		self.curr_theta = pose_message.theta #storing current angles
		if self.curr_theta < 0: #setting the current angle to absolute value if it becomes < zero
			self.curr_deg = 360 - abs(math.degrees(self.curr_theta))
		else:
			self.curr_deg = abs(math.degrees(self.curr_theta))
   
	def movCircle(self, ang_vel):
		obj_velocity_mssg = Twist() #creating objects
		obj_pose_mssg = Pose()

		start_degree = 0
		current_degree = 0

		handle_pub_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #publishing to a topic 
		lin_vel = 0.8
		
		max_degree = 360

		obj_velocity_mssg.linear.x = lin_vel # setting linear velocity to 0.8
		obj_velocity_mssg.angular.z = ang_vel


		while round(current_degree) < round(max_degree):#circle movement commands
			handle_pub_vel.publish(obj_velocity_mssg)

			current_degree = self.curr_deg - start_degree
			rospy.loginfo("Moving in a circle")

		obj_velocity_mssg.linear.x = 0
		obj_velocity_mssg.angular.z = 0
		handle_pub_vel.publish(obj_velocity_mssg) #stopping the turtle
        
        

def main():
	rospy.init_node('shapes_turtle', anonymous=True)         
	turtle = MoveTurtle() #calling the turtle movement code
	handle_sub_pose = rospy.Subscriber('/turtle1/pose', Pose, turtle.func_ros_sub_callback) #initially moving for completing the first circle

	turtle.movCircle(-0.8)
	
	turtle1 = MoveTurtle()
	handle_sub_pose1 = rospy.Subscriber('/turtle1/pose', Pose, turtle1.func_ros_sub_callback) # then moving the turtle for the second circle

	turtle1.movCircle(0.8)
	print("Goal Reached")

main()
