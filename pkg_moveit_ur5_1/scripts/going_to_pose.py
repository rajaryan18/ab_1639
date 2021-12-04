#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    


    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = -0.005722433368432027 #stare
    ur5_pose_1.position.y = 0.17338707533949152
    ur5_pose_1.position.z = 0.8250479925504336
    ur5_pose_1.orientation.x = 4.244282310144956e-05
    ur5_pose_1.orientation.y = -5.3660865475850754e-05
    ur5_pose_1.orientation.z =  0.8414480440681561
    ur5_pose_1.orientation.w =0.5403380279537919
    
    ur5_pose_2 = geometry_msgs.msg.Pose()   #attack
    ur5_pose_2.position.x = -0.12220546709698218
    ur5_pose_2.position.y =  0.4279221055311414
    ur5_pose_2.position.z = 1.0658527311141295
    ur5_pose_2.orientation.x = 0.4031472878359867
    ur5_pose_2.orientation.y = -0.25955826996077325
    ur5_pose_2.orientation.z =0.7390446221469114
    ur5_pose_2.orientation.w = 0.47416438902783586

    ur5_pose_3 = geometry_msgs.msg.Pose()    #close
    ur5_pose_3.position.x = -0.12226897500935444
    ur5_pose_3.position.y = 0.42788411404641413
    ur5_pose_3.position.z = 1.0658942331420729
    ur5_pose_3.orientation.x =0.2707781399596428
    ur5_pose_3.orientation.y = -0.39567920941517953
    ur5_pose_3.orientation.z = 0.8648044419019324
    ur5_pose_3.orientation.w =   0.1490987572827551
   
    ur5_pose_4 = geometry_msgs.msg.Pose()    #drop
    ur5_pose_4.position.x =  0.38662490787154535
    ur5_pose_4.position.y = 0.07852686944766107
    ur5_pose_4.position.z = 0.8251020222473139
    ur5_pose_4.orientation.x = -9.230204587803922e-06
    ur5_pose_4.orientation.y = -3.071055190296345e-05
    ur5_pose_4.orientation.z = 0.38945991695448035
    ur5_pose_4.orientation.w =  0.921043414860274

    ur5_pose_5 = geometry_msgs.msg.Pose()
    ur5_pose_5.position.x = 0.3866764233691064 #open
    ur5_pose_5.position.y = 0.07855342999801401
    ur5_pose_5.position.z = 0.8250482314384507
    ur5_pose_5.orientation.x = -1.3796947117108827e-05
    ur5_pose_5.orientation.y = 7.336513662366721e-06
    ur5_pose_5.orientation.z =  5.8682676985761346e-05
    ur5_pose_5.orientation.w = 0.9999999981560815


    ur5_pose_6 = geometry_msgs.msg.Pose()  #stare2
    ur5_pose_6.position.x = 0.10933924489325848
    ur5_pose_6.position.y = 0.23441498981436135
    ur5_pose_6.position.z = 0.8249877951707292
    ur5_pose_6.orientation.x = -1.9851702398970584e-05
    ur5_pose_6.orientation.y = 5.5017289038936074e-05
    ur5_pose_6.orientation.z = 0.6631200722964414
    ur5_pose_6.orientation.w = 0.7485130368247236
    
    ur5_pose_7 = geometry_msgs.msg.Pose()   #attack2
    ur5_pose_7.position.x = 0.12071215818452043
    ur5_pose_7.position.y =  0.5565832493353684
    ur5_pose_7.position.z = 0.8125704739292536
    ur5_pose_7.orientation.x = 0.2654114557763626
    ur5_pose_7.orientation.y = -0.28495440224973484
    ur5_pose_7.orientation.z = 0.6278787119302135
    ur5_pose_7.orientation.w = 0.6738887674431324
  
    ur5_pose_8 = geometry_msgs.msg.Pose()    #close2 X
    ur5_pose_8.position.x = 0.12072857898041675
    ur5_pose_8.position.y = 0.5565784124276567
    ur5_pose_8.position.z = 0.8125542544428371
    ur5_pose_8.orientation.x =0.13353736291001211
    ur5_pose_8.orientation.y = -0.36588593417410575
    ur5_pose_8.orientation.z = 0.8406614494265146
    ur5_pose_8.orientation.w = 0.3762759404063638
    
    ur5_pose_9 = geometry_msgs.msg.Pose()    #drop2
    ur5_pose_9.position.x =  0.3866666493309625
    ur5_pose_9.position.y = 0.07853917300855744
    ur5_pose_9.position.z = 0.8250645341169953
    ur5_pose_9.orientation.x = -4.329809074985927e-05
    ur5_pose_9.orientation.y = -9.89843103468303e-06
    ur5_pose_9.orientation.z = 0.3894375279776072
    ur5_pose_9.orientation.w =  0.9210528811257185
    
    ur5_pose_10 = geometry_msgs.msg.Pose()
    ur5_pose_10.position.x = 0.3866764233691064 #open
    ur5_pose_10.position.y = 0.07855342999801401
    ur5_pose_10.position.z = 0.8250482314384507
    ur5_pose_10.orientation.x = -1.3796947117108827e-05
    ur5_pose_10.orientation.y = 7.336513662366721e-06
    ur5_pose_10.orientation.z = 5.8682676985761346e-05
    ur5_pose_10.orientation.w = 0.9999999981560815
    
 #work in progress
    ur5_pose_11 = geometry_msgs.msg.Pose()  #stare2
    ur5_pose_6.position.x = 0.10933924489325848
    ur5_pose_6.position.y = 0.23441498981436135
    ur5_pose_6.position.z = 0.8249877951707292
    ur5_pose_6.orientation.x = -1.9851702398970584e-05
    ur5_pose_6.orientation.y = 5.5017289038936074e-05
    ur5_pose_6.orientation.z = 0.6631200722964414
    ur5_pose_6.orientation.w = 0.7485130368247236
    
    ur5_pose_7 = geometry_msgs.msg.Pose()   #attack2
    ur5_pose_7.position.x = 0.12071215818452043
    ur5_pose_7.position.y =  0.5565832493353684
    ur5_pose_7.position.z = 0.8125704739292536
    ur5_pose_7.orientation.x = 0.2654114557763626
    ur5_pose_7.orientation.y = -0.28495440224973484
    ur5_pose_7.orientation.z = 0.6278787119302135
    ur5_pose_7.orientation.w = 0.6738887674431324
  
    ur5_pose_8 = geometry_msgs.msg.Pose()    #close2 X
    ur5_pose_8.position.x = 0.12072857898041675
    ur5_pose_8.position.y = 0.5565784124276567
    ur5_pose_8.position.z = 0.8125542544428371
    ur5_pose_8.orientation.x =0.13353736291001211
    ur5_pose_8.orientation.y = -0.36588593417410575
    ur5_pose_8.orientation.z = 0.8406614494265146
    ur5_pose_8.orientation.w = 0.3762759404063638
    
    ur5_pose_9 = geometry_msgs.msg.Pose()    #drop2
    ur5_pose_9.position.x =  0.3866666493309625
    ur5_pose_9.position.y = 0.07853917300855744
    ur5_pose_9.position.z = 0.8250645341169953
    ur5_pose_9.orientation.x = -4.329809074985927e-05
    ur5_pose_9.orientation.y = -9.89843103468303e-06
    ur5_pose_9.orientation.z = 0.3894375279776072
    ur5_pose_9.orientation.w =  0.9210528811257185
    
    ur5_pose_10 = geometry_msgs.msg.Pose()
    ur5_pose_10.position.x = 0.3866764233691064 #open
    ur5_pose_10.position.y = 0.07855342999801401
    ur5_pose_10.position.z = 0.8250482314384507
    ur5_pose_10.orientation.x = -1.3796947117108827e-05
    ur5_pose_10.orientation.y = 7.336513662366721e-06
    ur5_pose_10.orientation.z = 5.8682676985761346e-05
    ur5_pose_10.orientation.w = 0.9999999981560815

    while not rospy.is_shutdown():
        
        ur5.go_to_pose(ur5_pose_1)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_2)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_3)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_4)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_5)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_6)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_7)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_8)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_9)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_10)
        rospy.sleep(2)
     

    del ur5


if __name__ == '__main__':
    main()

