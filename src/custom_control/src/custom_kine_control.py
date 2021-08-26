#! /usr/bin/env python

import sys
# sys.path is a list of absolute path strings
#sys.path.append('/lib') # <-- relative path
from lib_pose import *
from lib_custom_python_interface import MoveGroupPythonIntefaceTutorial
import numpy as np
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

custom = MoveGroupPythonIntefaceTutorial()

joints1 , poses1 = custom.go_to_joint_state(0,-pi/6,pi/3,0)
rospy.sleep(1)
print ("Current  Joint1: "+ str(joints1))
print ("Current  Poses1: "+ "\n" + str(np.around(poses1,decimals=2)))
""" 
custom.go_to_pose_goal(0.7,0,0.5,180,90,180)
rospy.sleep(1)
current_joints = custom.move_group.get_current_joint_values()
print ("Current  Joint: "+ str(current_joints))
current_pose = custom.move_group.get_current_pose()
print ("Current  Pose: "+ "\n" + str(current_pose.pose)) """

"""
cartesian_plan, fraction = ur5e.plan_cartesian_path(0.1,0.2,0.3)
ur5e.display_trajectory(cartesian_plan)
ur5e.execute_plan(cartesian_plan)

# Verify Pose and RPY
ur5e_pose = ur5e.move_group.get_current_pose()
pose1=xyzrpy(ur5e_pose)
print ("Current  Pose: "+ str(pose1))
ur5e_joints = ur5e.move_group.get_current_joint_values()
print ("Current  Joint: "+ str(ur5e_joints)) """