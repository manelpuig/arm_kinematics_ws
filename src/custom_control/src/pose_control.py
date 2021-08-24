#! /usr/bin/env python

import math
from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('custom_arm_control', anonymous=True)

robot = moveit_commander.RobotCommander()
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the joint values from the group and adjust some of the values:
pose_now=group.get_current_pose()

pose_goal = geometry_msgs.msg.Pose()
#pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.6
pose_goal.position.y = 0.0
pose_goal.position.z = 0.5

group.set_pose_target(pose_goal)
# you might want to try a different planner
# group.setPlannerId("RRTConnectkConfigDefault")
# sometimes the planner might find the path with more time available
#group.setPlanningTime(10);
plan = group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()

print ("Current Joint Values:")
joint_goal1=group.get_current_joint_values()
print ("q1 = "+str(round(math.degrees(joint_goal1[0]),1)))
print ("q2 = "+str(round(math.degrees(joint_goal1[1]),1)))
print ("q3 = "+str(round(math.degrees(joint_goal1[2]),1)))
print ("q4 = "+str(round(math.degrees(joint_goal1[3]),1)))

print ("Current Pose:")
print (pose_goal)
''' pose_goal1=group.get_current_pose()
print ("x = "+str(round(pose_goal1.position.x,1)))
print ("y = "+str(round(pose_goal1.position.y,1)))
print ("z = "+str(round(pose_goal1.position.z,1))) '''

moveit_commander.roscpp_shutdown()