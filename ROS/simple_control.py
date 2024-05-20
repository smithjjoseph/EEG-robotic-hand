#!/usr/bin/env python3
"""
:file: simple_control.py
:brief: Contains command line functions for opening and closing a qb softhand 2
\t This file is a modified version of supplied code
:TODO::
- Use `rosparam list [opt: namespace]` to get list of parameters 
- Consolidate magic values to more useful constants
  - Test with provided GUI and then use values in script
- Find out what JointTrajectoryPoint().time_from_start does
"""

import sys
import rospy
import numpy as np
from math import cos
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def close() -> None:
    """
    Sends message to close softhand
    :returns: `None`
    """
    # Declares that messages of type JointTrajectory are being posted to a specific topic with a limit of 10 async messages at a time
    pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, queue_size=10)
    # Creates a node called 'close_hand' to run process on
    rospy.init_node('close_hand')
    # Sets the sleep to fulfill a rate of 5Hz
    rate = rospy.Rate(5)
    for step in np.linspace(0, 3.142, 5):
        cmd = JointTrajectory()
        cmd.header.stamp = rospy.Time.now()
        cmd.joint_names = ['qbhand2m1_manipulation_joint', 'qbhand2m1_synergy_joint']
        pt = JointTrajectoryPoint()
        # Values are mapped to a sinusoid to make movement smooth
        # Meaning that the max speed is at the middle of the movement
        pt.positions = [0.0, 0.37*(cos(step+3.142)+1.01)]
        pt.velocities = [0.0, 0.0]
        pt.accelerations = [0.0, 0.0]
        pt.effort = [0.0, 0.0]
        pt.time_from_start = rospy.Time.from_sec(1)
        cmd.points.append(pt)
        pub.publish(cmd)
        rate.sleep()


def open() -> None:
    """
    Sends message to open softhand
    :returns: `None`
    """
    pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('open_hand')
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
        pub.publish(cmd)
        rate.sleep()


def close_man() -> None:
    """
    Sends message to close softhand manipulation joint
    :returns: `None`
    """
    # Declares that messages of type JointTrajectory are being posted to a specific topic with a limit of 10 async messages at a time
    pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, queue_size=10)
    # Creates a node called 'close_hand' to run process on
    rospy.init_node('close_hand')
    # Sets the sleep to fulfill a rate of 5Hz
    rate = rospy.Rate(5)
    for step in np.linspace(0, 3.142, 5):
        cmd = JointTrajectory()
        cmd.header.stamp = rospy.Time.now()
        cmd.joint_names = ['qbhand2m1_manipulation_joint', 'qbhand2m1_synergy_joint']
        pt = JointTrajectoryPoint()
        # Values are mapped to a sinusoid to make movement smooth
        # Meaning that the max speed is at the middle of the movement
        pt.positions = [0.37*(cos(step+3.142)+1.01), 0.0]
        pt.velocities = [0.0, 0.0]
        pt.accelerations = [0.0, 0.0]
        pt.effort = [0.0, 0.0]
        pt.time_from_start = rospy.Time.from_sec(1)
        cmd.points.append(pt)
        pub.publish(cmd)
        rate.sleep()


def open_man() -> None:
    """
    Sends message to open softhand manipulation joint
    :returns: `None`
    """
    pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('open_hand')
    rate = rospy.Rate(5)
    for step in np.linspace(-3.142, 0, 5):
        cmd = JointTrajectory()
        cmd.header.stamp = rospy.Time.now()
        cmd.joint_names = ['qbhand2m1_manipulation_joint', 'qbhand2m1_synergy_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.33*(cos(step+3.142)+1.01), 0.0]
        pt.velocities = [0.0, 0.0]
        pt.accelerations = [0.0, 0.0]
        pt.effort = [0.0, 0.0]
        pt.time_from_start = rospy.Time.from_sec(1)
        cmd.points.append(pt)
        pub.publish(cmd)
        rate.sleep()


def main() -> None:
    action: str = sys.argv[1]
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
    elif action == 'close_man':
        try:
            close_man()
        except rospy.ROSInterruptException:
            pass
    elif action == 'open_man':
        try:
            open_man()
        except rospy.ROSInterruptException:
            pass
    else:
        print("Incorrect usage.\npython3 softhand_control.py [open|close]")


if __name__ == '__main__':
    main()
