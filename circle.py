#!/usr/bin/env python3
import threading
import rospy
import math
import time
import sys

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

	def start_move(radius,ang_vel):
		start_degree = 0
		current_degree = 0
        handle_pub_vel = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)

        lin_vel = radius
    
        max_degree = 360

        obj_velocity_mssg.linear.x = lin_vel
        obj_velocity_mssg.angular.z = ang_vel

        print('Circle')

        while round(current_degree) < round(max_degree):
            handle_pub_vel.publish(obj_velocity_mssg)

            current_degree = self.curr_deg - start_degree
        obj_velocity_mssg.linear.x = 0
        obj_velocity_mssg.angular.z = 0
            
        handle_pub_vel.publish(obj_velocity_mssg)
        print('Circle Complete!')

	def movCircle(self,radius):
		obj_velocity_mssg = Twist()
        obj_pose_mssg = Pose()

        c1 = threading.Thread(target = start_move, args= (radius,-0.8))
        c2 = threading.Thread(target = start_move, args= (radius, 0.8))
        
        c1.start()
        c1.join()
        c2.start()
        c2.join()

def main():
    rospy.init_node('shapes_turtle', anonymous=True)         
    turtle = MoveTurtle()
    handle_sub_pose = rospy.Subscriber('/turtle1/pose', Pose, turtle.func_ros_sub_callback)

    turtle.movCircle(float(sys.argv[1]))

main()
