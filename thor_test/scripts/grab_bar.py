#!/usr/bin/env python

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function
from future import standard_library

import sys, rospy, tf, moveit_commander, random
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi

from thor_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse 

#Script to pick a bar, go home and place it in its original position.


#End effector orientations
orient = Quaternion(*tf.transformations.quaternion_from_euler(pi, 0, 0))
orient_rotated = Quaternion(*tf.transformations.quaternion_from_euler(pi/2, 0, 0))

#Predefined poses for the end effector
pre_grasp = Pose(Point( 0.4, 0, 0.15), orient)
grasp = Pose(Point( 0.45, 0, 0.15), orient)
lift = Pose(Point( 0.45, 0, 0.3), orient)
rotate = Pose(Point( 0.45, 0, 0.3), orient_rotated)
place = Pose(Point( 0.45, 0, 0.15), orient)




#Start Moveit
moveit_commander.roscpp_initialize(sys.argv)
#Start Node
rospy.init_node('thor_rotate_arm',anonymous=True)
#Commander for the arm and for the robot
group = moveit_commander.MoveGroupCommander("arm")
robot = moveit_commander.RobotCommander()


#Service to control the gripper
gripper = rospy.ServiceProxy("close_gripper", GripperControl)
#Request Message to open gripper
open_gripper=GripperControlRequest()
open_gripper.close=False
#Request Message to close gripper
close_gripper=GripperControlRequest()
close_gripper.close=True
#Response for gripper service calls
gripper_response=GripperControlResponse()


# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print("============ Reference frame: %s", planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print("============ End effector: %s", eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Robot Groups:", robot.get_group_names())

'''
# Sometimes for debugging it is useful to print the entire state of the
# robot:
print ("============ Printing robot state")
print (robot.get_current_state())
print ("")
'''

#Set Velocity
group.set_max_velocity_scaling_factor(1)

#Open Gripper
rospy.loginfo("Opening Gripper")
gripper_response=gripper(open_gripper)
print(gripper_response.success)  

# Rotation Sequence
while not rospy.is_shutdown():


    #Home
    if not rospy.is_shutdown():
      rospy.loginfo("Going Home")    
      group.set_named_target("Home")
      print(group.go(wait=True)) 
    
    #Pre Grasp Position
    if not rospy.is_shutdown():
      rospy.loginfo("Pre Grasp")
      group.set_pose_target(pre_grasp)
      print(group.go(wait=True))

    #Grasp Position
    if not rospy.is_shutdown():
      rospy.loginfo("Grasping")
      group.set_pose_target(grasp)
      print(group.go(wait=True))

    #Close Gripper
    if not rospy.is_shutdown():
      rospy.loginfo("Closing Gripper")
      gripper_response=gripper(close_gripper)
      print(gripper_response.success)
      rospy.sleep(1)    

    #Lift Position
    if not rospy.is_shutdown():
      rospy.loginfo("Lifting")
      group.set_max_velocity_scaling_factor(1)
      group.set_pose_target(lift)
      print(group.go(wait=True))

    '''
    #Rotate Bar
    if not rospy.is_shutdown():
      rospy.loginfo("Rotating")
      group.set_pose_target(rotate)
      print(group.go(wait=True))'''
   


    #Place Position
    if not rospy.is_shutdown():
      rospy.loginfo("Placing")
      group.set_pose_target(place)
      print(group.go(wait=True))

    
    #Open Gripper
    if not rospy.is_shutdown():
      rospy.loginfo("Opening Gripper")
      gripper_response=gripper(open_gripper)
      print(gripper_response.success)  
      

    #Pre Grasp Position
    if not rospy.is_shutdown():
      rospy.loginfo("Pre Grasp")
      group.set_max_velocity_scaling_factor(1)
      group.set_pose_target(pre_grasp)
      print(group.go(wait=True))

    break


#Home
rospy.loginfo("Going Home - Shutdown")    
group.set_named_target("Home")
print(group.go(wait=True)) #Returns true if pose was achieved

#group.set_named_target("Target") -> Para ir a posiciones predefinidas


moveit_commander.roscpp_shutdown()
