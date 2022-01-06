#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import numpy as np
import tf2_ros

class Ur5Moveit:

    # Constructor
	def __init__(self):

		rospy.init_node('node_eg3_set_joint_angles', anonymous=True)
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander()
		self._scene = moveit_commander.PlanningSceneInterface()

		#dictionary of all planning groups to be used
		self._group = {'arm_control': moveit_commander.MoveGroupCommander('arm_control'), 'ur5_1_planning_group': moveit_commander.MoveGroupCommander('ur5_1_planning_group'), 'wrist_control': moveit_commander.MoveGroupCommander('wrist_control')}
		
		self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

		self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()
		self._group_names = self._robot.get_group_names()
		self._box_name = 'agribot'

        # Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()

		rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def go_to_pose(self, arg_pose, groupName):
		self._planning_frame = self._group[groupName].get_planning_frame()
		self._eef_link = self._group[groupName].get_end_effector_link()
		pose_values = self._group[groupName].get_current_pose().pose
		#rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
		#rospy.loginfo(pose_values)

		a = self._group[groupName].set_pose_target(arg_pose)
		#rospy.loginfo(a)
		self._group[groupName].set_goal_tolerance(0.01)
		s =  self._group[groupName].plan()
		if len(s[1].joint_trajectory.points) > 0:
			return s[1].joint_trajectory.points[len(s[1].joint_trajectory.points)-1].positions			
			
	def go_to_predefined_pose(self, arg_pose_name, groupName):
		self._planning_frame = self._group[groupName].get_planning_frame()
		self._eef_link = self._group[groupName].get_end_effector_link()
		rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
		self._group[groupName].set_named_target(arg_pose_name)
		#self._group[groupName].set_goal_tolerance(0.01)
		plan = self._group[groupName].plan()
		goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
		goal.trajectory = plan
		flag_plan = self._group[groupName].go(wait=True)
		rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
	
	def set_joint_angles(self, arg_list_joint_angles):
		self._group['arm_control'].set_joint_value_target(arg_list_joint_angles)
		self._group['arm_control'].plan()
		flag_plan = self._group['arm_control'].go(wait=True)


    # Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def backToOrigin(ur5):
	ur5.go_to_predefined_pose('closed', 'ur5_1_planning_group') #capture the tomato
	rospy.sleep(1)
	ur5.go_to_predefined_pose('allZero', 'arm_control') #go back to the initial pose
	rospy.sleep(1)
	ur5.go_to_predefined_pose('open', 'ur5_1_planning_group') #dropping the tomato in the basket


def main():
	ur5 = Ur5Moveit()
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	ur5.go_to_predefined_pose('camera_pose', 'arm_control')
	rospy.sleep(1)

	detections = []

	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform('ebot_base', 'camera_rgb_frame2', rospy.Time.now(), rospy.Duration(1.0))
			x_cod = trans.transform.translation.x
			y_cod = trans.transform.translation.y
			z_cod = trans.transform.translation.z
			flag = True
			for det in detections:
				if ((x_cod - det[0]) **2) + ((y_cod - det[1])**2) + ((z_cod - det[2])**2) < 0.5:
					flag = False
			if flag == True: 
				ur5.go_to_predefined_pose('shoulder_move', 'arm_control')
            			ur5_pose_1 = geometry_msgs.msg.Pose()
            			ur5_pose_1.position.x = x_cod + 0.05
            			ur5_pose_1.position.y = y_cod + 0.2
            			ur5_pose_1.position.z = z_cod
            			ur5_pose_1.orientation.x = euler_to_quaternion(3.14, 0, -3.14)[0]
            			ur5_pose_1.orientation.y = euler_to_quaternion(3.14, 0, -3.14)[1]
            			ur5_pose_1.orientation.z = euler_to_quaternion(3.14, 0, -3.14)[2]
            			ur5_pose_1.orientation.w = euler_to_quaternion(3.14, 0, -3.14)[3]
            			joint_list = ur5.go_to_pose(ur5_pose_1, 'arm_control')
            			ur5.set_joint_angles(joint_list)
            			rospy.sleep(1)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			pass


	#rospy.sleep(1)
	#backToOrigin(ur5)

	del ur5


if __name__ == '__main__':
	main()
