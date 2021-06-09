#!/usr/bin/env python3

#    spawn_box.py: Script to spawn a long and straight bar in a gazebo world
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

#ROS and standard imports
import sys, rospy, tf
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy


if __name__ == '__main__':
  rospy.init_node("spawn_bar")
  rospy.wait_for_service("gazebo/delete_model")
  rospy.wait_for_service("gazebo/spawn_sdf_model")
  delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
  delete_model("box")
  s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
  orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
  box_pose = Pose(Point(0.3,0,0.011), orient)
  unit = 0.05
  with open("box.sdf", "r") as f:
    box_sdf = f.read()

  print(s("box", box_sdf, "", box_pose, "world"))

  
