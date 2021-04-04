#!/usr/bin/env python3
import sys, rospy, tf
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy

#Script to spawn a long and thin bar in a gazebo world
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

  
