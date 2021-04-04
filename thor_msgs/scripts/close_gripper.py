#!/usr/bin/env python3
# This code has been adapted from the ROS Wiki ROS Service tutorials and the HRWROS MOOC
# (http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

from thor_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse

import sys, rospy, tf, moveit_commander, random
import rospy
import numpy as np


#Service server to control a simple parallel gripper with Moveit

class control_gripper:
    def __init__(self):




        #Initialize Moveit
        moveit_commander.roscpp_initialize(sys.argv)
        

        # Create a ROS service type.
        self.service = rospy.Service('close_gripper', GripperControl, self.control_callback)

        # Log message about service availability.
        rospy.loginfo('Close Gripper Service is now available.')

        #Create the group commander for the gripper
        self.group = moveit_commander.MoveGroupCommander("gripper")




    # Service callback function.
    def control_callback(self,req):
        rospy.loginfo('Close Gripper Service Called.')


        # Instantiate the response message object.
        res = GripperControlResponse()
            

        if(req.close):
          self.group.set_named_target("close")
          res= not self.group.go(wait=True) #Negated because failure in the go function means some object stopped the gripper
        else:
          self.group.set_named_target("open")
          res=self.group.go(wait=True)
                


        #Return the response message.
        return res


    

if __name__ == "__main__":
    rospy.init_node('gripper_control_server', anonymous = False)
    server=control_gripper()
    rospy.spin()

