#! /usr/bin/env python

import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

pose_arm=[0,0,0,0,0,0]

def pose_target_callback(data):
    group.set_pose_target(data)
    group.plan()
    #rospy.sleep(1)
    group.go(wait=True)
    #rospy.sleep(1)
    rospy.loginfo("Robot moving to target\n")

def move_arm():
    global pose_arm
    pub = rospy.Publisher('/pose_current', PoseStamped, queue_size=10)
    rospy.Subscriber('/pose_target', Pose, pose_target_callback)
    rate = rospy.Rate(10) # 10hz
    pose_current = PoseStamped()
    while not rospy.is_shutdown():
        print ("Current Pose:")
        pose_current=group.get_current_pose()
        pose_arm[0]=round(pose_current.pose.position.x,2)
        pose_arm[1]=round(pose_current.pose.position.y,2)
        pose_arm[2]=round(pose_current.pose.position.z,2)
        qx=pose_current.pose.orientation.x
        qy=pose_current.pose.orientation.y
        qz=pose_current.pose.orientation.z
        qw=pose_current.pose.orientation.w
        quat2=[qx, qy, qz, qw]
        (roll, pitch, yaw)=euler_from_quaternion(quat2)
        pose_arm[3]=round(math.degrees(roll),1)
        pose_arm[4]=round(math.degrees(pitch),1)
        pose_arm[5]=round(math.degrees(yaw),1)
        rospy.loginfo("Robot arm current Pose= %s\n",pose_arm)

        pub.publish(pose_current)
        rate.sleep()

if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5e_node', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()    
        group = moveit_commander.MoveGroupCommander("Arm")
        #d= rospy.get_param("~d")
        move_arm()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass