#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897


def circle(speed,radius):

	print "If you want to quit Ctl-c me"

	vel_msg.linear.x = speed
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = speed/radius
	
	#Move Robot in circle
	while not rospy.is_shutdown():
		velocity_publisher.publish(vel_msg)
	
	vel_msg.linear.x = 0	
	vel_msg.linear.z = 0
	velocity_publisher.publish(vel_msg)	

def move(speed,distance,isForward,topic):
   
    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    temp = True
    while (temp):

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
	    rospy.loginfo(topic)
	temp = False
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)

def rotate(speed,angle,clockwise):

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)


    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    

if __name__ == '__main__':
    try:
       
	#Starts a new node
	rospy.init_node('shapes', anonymous=True)
    	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()
	print("Let's move your robot")
    	speed = input("Input your speed:")
    	distance = input("Type your distance:")
    	isForward = input("Foward?: ")#True or False
	print("which shape you want to draw?")
	sides = input("Enter number of sides: ")
	if sides==0:
		circle(speed,distance)
	else:
		for i in range(sides):
			#print("Making shape of side: ",sides)
			topic = "making your shape of side " + str(sides)
			move(speed,distance,isForward,topic)
			angle = 360/sides
			rotate(50,angle,0)
	print("DONE!")
    except rospy.ROSInterruptException: pass

