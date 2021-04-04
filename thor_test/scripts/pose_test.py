#!/usr/bin/env python3

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function
from future import standard_library

import sys, rospy, tf, moveit_commander, random
import pandas as pd
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi

from thor_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse 

#Script to test different poses for the end effector and record success (planning and execution) and accuracy

def pose_test():

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

    #Set Velocity
    group.set_max_velocity_scaling_factor(1)

    #Set planning time
    group.set_planning_time(0.5) #Fast Timeout
    
    

  
    #End effector orientations (RPY)
    orient_front = [pi, 0, 0]
    orient_down = [pi, pi/2, 0]
    orient_incline = [pi, pi/4, 0]

   

    #Bound of the experiment

    #Tested Orientations
    #orientations=[orient_front, orient_down, orient_incline]
    orientations=[orient_down]
    

    #x axis    
    x_min=0
    x_max=0.3
    #y axis (Uses negative values to explore right and left sides of the robot)
    y_min=-0.5
    y_max=0.5
    #z_axis 
    z_min=0.02
    z_max=0.1

    #Resolution of the experiment
    x_divide=3
    y_divide=3
    z_divide=3


    


    #List of points
    x_points=np.linspace(x_min,x_max, num=x_divide)

    
    y_points=np.linspace(y_min,y_max, num=y_divide)
    #Using only y=0
    #y_points=[0]

    z_points=np.linspace(z_min,z_max, num=z_divide)
        
    poses=[]
    
    

     
    #Generate a list of all the poses of the experiment
    for x in x_points:
        for y in y_points:
            for z in z_points:
                for r in orientations:
                    #Create a pose in the x,y,z,R,P,Y format
                    pose=[x, y, z, r[0],r[1],r[2]]
                    #Append the pose to the list of poses                    
                    poses.append(pose)



    #Shuffle of the poses to avoid bias in the experiment
    random.shuffle(poses)

    results=[]

    
    #Start in home
    rospy.loginfo("Going Home")    
    group.set_named_target("Home")
    
    print(group.go(wait=True)) 
    
    #Open Gripper
    rospy.loginfo("Opening Gripper")
    gripper_response=gripper(open_gripper)
    print(gripper_response.success) 
    
    #Result format: [Setpoint_x,Setpoint_y,Setpoint_z,Setpoint_R,Setpoint_P,Setpoint_Y,plan_success,/
                    #exec_success,reached_x,reached_y,reached_z,reached_R,reached_P,reached_Y]
    


    rospy.loginfo("Starting Experiment...")
    for i in poses:
        result=[]
        #Build pose message
        target_pose=Pose(Point(i[0],i[1],i[2]),Quaternion(*tf.transformations.quaternion_from_euler(i[3], i[4], i[5])))
        #Plan Pose
        group.set_pose_target(target_pose)
        rospy.loginfo("Planing")
        plan=group.plan()

        #If planning was successfull, exectute trajectory
        if plan[0]: #modificada la funcion plan() en noetic

            rospy.loginfo("Executing Pose")
            exec_success=group.execute(plan[1],wait=True)

            #Called two times because of syncronization issues
            eff_pose=group.get_current_pose()
            rospy.sleep(0.1)
            eff_pose=group.get_current_pose()

            eff_pos=eff_pose.pose.position
      
            eff_quat=eff_pose.pose.orientation

            eff_orient=tf.transformations.euler_from_quaternion([eff_quat.x,eff_quat.y,eff_quat.z,eff_quat.w])

            result=[i[0],i[1],i[2],i[3], i[4], i[5],True,exec_success,eff_pos.x,eff_pos.y,eff_pos.z,eff_orient[0],eff_orient[1],eff_orient[2]]
            
            

        else:
            rospy.loginfo("Failed Plan")
            result=[i[0],i[1],i[2],i[3], i[4], i[5],False,False,0,0,0,0,0,0]

        #Store Results
        rospy.loginfo("Storing Result")
        results.append(result)
    

    rospy.loginfo("Saving Results")
    results_df=pd.DataFrame(results,
        columns=["x_set","y_set","z_set","R_set","P_set","Y_set","plan","execute","x_reach","y_reach","z_reach","R_reach","P_reach","Y_reach"])

    results_df.to_csv('test.csv', index=False)
          

    


  
        
    


    

    
    

    #Shut down moveit
    moveit_commander.roscpp_shutdown()
    


if __name__=='__main__':
  try:
    pose_test()
  except rospy.ROSInterruptException:
    pass

