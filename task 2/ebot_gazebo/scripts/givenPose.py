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


class Ur5Moveit:

    # Constructor
	def __init__(self):

		rospy.init_node('node_eg3_set_joint_angles', anonymous=True)
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander()
		self._scene = moveit_commander.PlanningSceneInterface()

		#dictionary of all planning groups to be used
		self._group = {'arm_control': moveit_commander.MoveGroupCommander('arm_control'), 'ur5_1_planning_group': moveit_commander.MoveGroupCommander('ur5_1_planning_group')}
		
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
		rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
		rospy.loginfo(pose_values)

		self._group[groupName].set_pose_target(arg_pose)
		#self._group[groupName].set_goal_tolerance(0.01)
		self._group[groupName].plan()
		flag_plan = self._group[groupName].go(wait=True)  # wait=False for Async Move

		pose_values = self._group[groupName].get_current_pose().pose
		rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		rospy.loginfo(pose_values)

		list_joint_values = self._group[groupName].get_current_joint_values()
		rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		rospy.loginfo(list_joint_values)

		if (flag_plan == True):
			rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
		else:
			rospy.logerr('\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')
			
			
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


    # Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
		
		
def euler_to_quaternion(yaw, pitch, roll):
	#function to convert euler angles to quaternion
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qx, qy, qz, qw]


def backToOrigin(ur5):
	ur5.go_to_predefined_pose('closed', 'ur5_1_planning_group') #capture the tomato
	rospy.sleep(1)
	ur5.go_to_predefined_pose('allZero', 'arm_control') #go back to the initial pose
	rospy.sleep(1)
	ur5.go_to_predefined_pose('open', 'ur5_1_planning_group') #dropping the tomato in the basket


def main():
	ur5 = Ur5Moveit()
	ur5.go_to_predefined_pose('shoulder_move', 'arm_control')
	rospy.sleep(1)

	ur5_pose_1 = geometry_msgs.msg.Pose()
	ur5_pose_1.position.x = 0.1117
	ur5_pose_1.position.y = 0.2844
	ur5_pose_1.position.z = 0.944534
	ur5_pose_1.orientation.x = euler_to_quaternion(2.9, 1.57, -3.1)[0]
	ur5_pose_1.orientation.y = euler_to_quaternion(2.9, 1.57, -3.1)[1]
	ur5_pose_1.orientation.z = euler_to_quaternion(2.9, 1.57, -3.1)[2]
	ur5_pose_1.orientation.w = euler_to_quaternion(2.9, 1.57, -3.1)[3]

	ur5_pose_2 = geometry_msgs.msg.Pose()
	ur5_pose_2.position.x = 0.0127
	ur5_pose_2.position.y = 0.3544
	ur5_pose_2.position.z = 0.944534
	ur5_pose_2.orientation.x = euler_to_quaternion(2.9, 1, -3.1)[0]
	ur5_pose_2.orientation.y = euler_to_quaternion(2.9, 1, -3.1)[1]
	ur5_pose_2.orientation.z = euler_to_quaternion(2.9, 1, -3.1)[2]
	ur5_pose_2.orientation.w = euler_to_quaternion(2.9, 1, -3.1)[3]
	ur5.go_to_pose(ur5_pose_2, 'arm_control')

	rospy.sleep(1)
	backToOrigin(ur5)

	ur5.go_to_predefined_pose('shoulder_move', 'arm_control')
	rospy.sleep(1)

	ur5_pose_1 = geometry_msgs.msg.Pose()
	ur5_pose_1.position.x = 0.0377
	ur5_pose_1.position.y = 0.281044
	ur5_pose_1.position.z = 0.714534
	ur5_pose_1.orientation.x = euler_to_quaternion(-2.8, 0, 2)[0]
	ur5_pose_1.orientation.y = euler_to_quaternion(-2.8, 0, 2)[1]
	ur5_pose_1.orientation.z = euler_to_quaternion(-2.8, 0, 2)[2]
	ur5_pose_1.orientation.w = euler_to_quaternion(-2.8, 0, 2)[3]
	ur5.go_to_pose(ur5_pose_1, 'arm_control')

	ur5_pose_2 = geometry_msgs.msg.Pose()
	ur5_pose_2.position.x = 0.0377
	ur5_pose_2.position.y = 0.281044
	ur5_pose_2.position.z = 0.6604534
	ur5_pose_2.orientation.x = euler_to_quaternion(-2.8, 0, -3.1)[0]
	ur5_pose_2.orientation.y = euler_to_quaternion(-2.8, 0, -3.1)[1]
	ur5_pose_2.orientation.z = euler_to_quaternion(-2.8, 0, -3.1)[2]
	ur5_pose_2.orientation.w = euler_to_quaternion(-2.8, 0, -3.1)[3]
	ur5.go_to_pose(ur5_pose_2, 'arm_control')

	rospy.sleep(0.5)
	backToOrigin(ur5)

	ur5.go_to_predefined_pose('shoulder_move', 'arm_control')
	rospy.sleep(1)

	ur5_pose_1 = geometry_msgs.msg.Pose()
	ur5_pose_1.position.x = 0.0417
	ur5_pose_1.position.y = 0.2014
	ur5_pose_1.position.z = 1.184534
	ur5_pose_1.orientation.x = euler_to_quaternion(-2.2, -1, -3.1)[0]
	ur5_pose_1.orientation.y = euler_to_quaternion(-2.2, -1, -3.1)[1]
	ur5_pose_1.orientation.z = euler_to_quaternion(-2.2, -1, -3.1)[2]
	ur5_pose_1.orientation.w = euler_to_quaternion(-2.2, -1, -3.1)[3]

	ur5_pose_2 = geometry_msgs.msg.Pose()
	ur5_pose_2.position.x = 0.0417
	ur5_pose_2.position.y = 0.30104
	ur5_pose_2.position.z = 1.184534
	ur5_pose_2.orientation.x = euler_to_quaternion(-2.2, 0, -3.14)[0]
	ur5_pose_2.orientation.y = euler_to_quaternion(-2.2, 0, -3.14)[1]
	ur5_pose_2.orientation.z = euler_to_quaternion(-2.2, 0, -3.14)[2]
	ur5_pose_2.orientation.w = euler_to_quaternion(-2.2, 0, -3.14)[3]
	ur5.go_to_pose(ur5_pose_2, 'arm_control')

	rospy.sleep(0.5)
	backToOrigin(ur5)

	del ur5


if __name__ == '__main__':
	main()
