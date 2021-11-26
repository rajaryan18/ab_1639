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

		self._planning_group = "arm_control"
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander()
		self._scene = moveit_commander.PlanningSceneInterface()
		self._group = moveit_commander.MoveGroupCommander(self._planning_group)
		self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

		self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = 'agribot'

        # Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()

		rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def go_to_pose(self, arg_pose):

		pose_values = self._group.get_current_pose().pose
		rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
		rospy.loginfo(pose_values)

		self._group.set_pose_target(arg_pose)
		self._group.set_goal_tolerance(0.01)
		self._group.plan()
		flag_plan = self._group.go(wait=True)  # wait=False for Async Move

		pose_values = self._group.get_current_pose().pose
		rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		rospy.loginfo(pose_values)

		list_joint_values = self._group.get_current_joint_values()
		rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		rospy.loginfo(list_joint_values)

		if (flag_plan == True):
			rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
		else:
			rospy.logerr('\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')
			
			
	def go_to_predefined_pose(self, arg_pose_name):
		rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
		self._group.set_named_target(arg_pose_name)
		plan = self._group.plan()
		goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
		goal.trajectory = plan
		flag_plan = self._group.go(wait=True)
		rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')


    # Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
		
		
def euler_to_quaternion(yaw, pitch, roll):

	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qx, qy, qz, qw]


def main():

	ur5 = Ur5Moveit()	
	
	ur5.go_to_predefined_pose("shoulder_move")
	rospy.sleep(2)
	ur5_pose_1 = geometry_msgs.msg.Pose()
	ur5_pose_1.position.x = 0.0327
	ur5_pose_1.position.y = 0.301044
	ur5_pose_1.position.z = 0.874534
	ur5_pose_1.orientation.x = euler_to_quaternion(-2.7, -0.0024, -3.1)[0]
	ur5_pose_1.orientation.y = euler_to_quaternion(-2.7, -0.0024, -3.1)[1]
	ur5_pose_1.orientation.z = euler_to_quaternion(-2.7, -0.0024, -3.1)[2]
	ur5_pose_1.orientation.w = euler_to_quaternion(-2.7, -0.0024, -3.1)[3]
	ur5.go_to_pose(ur5_pose_1)
	
	ur5_pose_2 = geometry_msgs.msg.Pose()
	ur5_pose_2.position.x = 0.0327
	ur5_pose_2.position.y = 0.301044
	ur5_pose_2.position.z = 0.674534
	ur5_pose_2.orientation.x = euler_to_quaternion(-2.7, -0.0024, -3.1)[0]
	ur5_pose_2.orientation.y = euler_to_quaternion(-2.7, -0.0024, -3.1)[1]
	ur5_pose_2.orientation.z = euler_to_quaternion(-2.7, -0.0024, -3.1)[2]
	ur5_pose_2.orientation.w = euler_to_quaternion(-2.7, -0.0024, -3.1)[3]
	ur5.go_to_pose(ur5_pose_2)

	del ur5


if __name__ == '__main__':
	main()
