#! /usr/bin/env python

import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import copy
import rospy

def xyzrpy(current_pose):
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

def main():
    try:
        pose1=xyzrpy(ur5e_pose)
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()