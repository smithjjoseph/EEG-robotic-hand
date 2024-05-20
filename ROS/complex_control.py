#!/usr/bin/env python3
"""
:file: complex_control.py
:brief: Contains functions for movement control of a qb softhand 2
:author: Joseph Smith
"""

import rospy
from numbers import Real
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class HandControl:
    def __init__(self) -> None:
        self.synergy_joint_pos: float = 0.0
        # self.reset()

        self.pub = rospy.Publisher(
            '/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command',
            JointTrajectory, queue_size=10)
        rospy.init_node('move_hand')
        self.rate = rospy.Rate(5)


    def get_synergy_joint_pos(self) -> float:
        """Getter method for the synergy joint servo position
        
        :returns Float: Returns a float between 0 and 1 signifying servo
        \t position
        """
        return self.synergy_joint_pos


    def reset(self, open: bool) -> None:
        """At class initialisation either open or close the hand fully so that
        the starting position is known \n
        This may not be necessary if there is a method for getting the hand's
        joint values

        :param open: True if hand should start fully opened,
        \t False if the hand should start fully closed 
        :type open: bool
        :returns: None
        """
        # Won't have to implement this if the joint state feature works
        raise NotImplemented()


    # TODO:: Add duration parameter to this function
    def movement(self, percent_closed: Real) -> None:
        """Makes a movement based on how closed the hand should be

        :param percent_closed: Real number between 0 and 100 representing the
        \t percentage that the softhand should be closed
        :type percent_closed: Real
        :returns: None
        """
        float_closed = max(0, min(100, percent_closed)) / 100

        pt = JointTrajectoryPoint()
        pt.positions = [0.0, float_closed]
        pt.velocities = [0.0, 0.0]
        pt.accelerations = [0.0, 0.0]
        pt.effort = [0.0, 0.0]
        pt.time_from_start = rospy.Time.from_sec(0.2)

        cmd = JointTrajectory()
        cmd.header.stamp = rospy.Time.now()
        cmd.joint_names = ['qbhand2m1_manipulation_joint', 'qbhand2m1_synergy_joint']
        cmd.points.append(pt)

        self.pub.publish(cmd)
        self.rate.sleep()

        self.synergy_joint_pos = float_closed


if __name__ == '__main__':
    import time
    import random

    control = HandControl()

    # Test movement for 5 random positions
    for i in range(5):
        control.movement(random.randint(1,100))
        time.sleep(1)
