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
joint_goal = group.get_current_joint_values()

joint_goal[0] = math.radians(45)
joint_goal[1] = -pi/3
joint_goal[2] = pi/2
joint_goal[3] = math.radians(0)
group.set_joint_value_target(joint_goal)

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
plan = group.plan()
group.go(joint_goal, wait=True)
group.execute(plan, wait=True)
rospy.sleep(1)
# Calling ``stop()`` ensures that there is no residual movement
group.stop()

print ("Current Joint Values:")
joint_goal1=group.get_current_joint_values()
print ("q1 = "+str(round(math.degrees(joint_goal1[0]),1)))
print ("q2 = "+str(round(math.degrees(joint_goal1[1]),1)))
print ("q3 = "+str(round(math.degrees(joint_goal1[2]),1)))
print ("q4 = "+str(round(math.degrees(joint_goal1[3]),1)))

print ("Current Pose:")
pose_goal=group.get_current_pose()
print ("x = "+str(round(pose_goal.pose.position.x,2)))
print ("y = "+str(round(pose_goal.pose.position.y,1)))
print ("z = "+str(round(pose_goal.pose.position.z,1)))

moveit_commander.roscpp_shutdown()