#!/usr/bin/env python3

#    close_gripper.py: Service server to control a simple parallel gripper with Moveit
#    Copyright (C) 2021  Emanuel Fallas (efallashdez@gmail.com)

#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or any later version.

#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#Thor messages imports
from thor_msgs.srv import GripperControl, GripperControlRequest, GripperControlResponse

#ROS and standard imports
import sys, rospy, tf, moveit_commander, random
import rospy
import numpy as np


#MAIN CLASS
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

#MAIN PROGRAM: Creates a control_gripper object and spins the thread
if __name__ == "__main__":
    rospy.init_node('gripper_control_server', anonymous = False)
    server=control_gripper()
    rospy.spin()

