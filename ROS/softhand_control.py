#!/usr/bin/env python

import sys
import rospy
import numpy as np
from math import cos
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def close():
    pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('demo', anonymous=True)
    rate = rospy.Rate(5)
    for step in np.linspace(0, 3.142, 5):
        cmd = JointTrajectory()
        cmd.header.stamp = rospy.Time.now()
        cmd.joint_names = ['qbhand2m1_manipulation_joint', 'qbhand2m1_synergy_joint']
        pt = JointTrajectoryPoint()
        # pt.positions = [0.0, 0.33*(cos(step+3.142)+1.01)]
        pt.positions = [0.0, 0.37*(cos(step+3.142)+1.01)]
        pt.velocities = [0.0, 0.0]
        pt.accelerations = [0.0, 0.0]
        pt.effort = [0.0, 0.0]
        pt.time_from_start = rospy.Time.from_sec(1)
        cmd.points.append(pt)
        # rospy.loginfo(cmd)
        pub.publish(cmd)
        # print(pt.positions)
        rate.sleep()


def open():
    pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('demo', anonymous=True)
    rate = rospy.Rate(5)
    for step in np.linspace(-3.142, 0, 5):
        cmd = JointTrajectory()
        cmd.header.stamp = rospy.Time.now()
        cmd.joint_names = ['qbhand2m1_manipulation_joint', 'qbhand2m1_synergy_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, 0.33*(cos(step+3.142)+1.01)]
        pt.velocities = [0.0, 0.0]
        pt.accelerations = [0.0, 0.0]
        pt.effort = [0.0, 0.0]
        pt.time_from_start = rospy.Time.from_sec(1)
        cmd.points.append(pt)
        # rospy.loginfo(cmd)
        pub.publish(cmd)
        # print(pt.positions)
        rate.sleep()


def main() -> None:
    action = sys.argv[1]
    if action == 'close':
        try:
            close()
        except rospy.ROSInterruptException:
            pass
    elif action == 'open':
        try:
            open()
        except rospy.ROSInterruptException:
            pass
    else:
        print("Incorrect usage.\npython3 softhand_control.py [open|close]")


if __name__ == '__main__':
    main()
