#! /usr/bin/env python

import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import copy
import rospy

def current_xyzrpy(current_pose):
    # current_pose type PoseStamped
    x=current_pose.pose.position.x
    y=current_pose.pose.position.y
    z=current_pose.pose.position.z
    qx=current_pose.pose.orientation.x
    qy=current_pose.pose.orientation.y
    qz=current_pose.pose.orientation.z
    qw=current_pose.pose.orientation.w
    quat=[qx, qy, qz, qw]
    (roll, pitch, yaw)=euler_from_quaternion(quat)
    r=math.degrees(roll)
    p=math.degrees(pitch)
    w=math.degrees(yaw)
    current_pose_vector=[x,y,z,r,p,w]
    return current_pose_vector

def target_xyzrpy(target_pose):
    # target_pose type Pose
    x=target_pose.position.x
    y=target_pose.position.y
    z=target_pose.position.z
    qx=target_pose.orientation.x
    qy=target_pose.orientation.y
    qz=target_pose.orientation.z
    qw=target_pose.orientation.w
    quat=[qx, qy, qz, qw]
    (roll, pitch, yaw)=euler_from_quaternion(quat)
    r=math.degrees(roll)
    p=math.degrees(pitch)
    w=math.degrees(yaw)
    target_pose_vector=[x,y,z,r,p,w]
    return target_pose_vector

def main():
    try:
        pose1=xyzrpy(ur5e_pose)
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()