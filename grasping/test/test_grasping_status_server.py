#!/usr/bin/env python

"""
Unit test for the grasping_status_server.py.

NOTE: This should be run via 'rosrun grasping test_grasping_status_server.py' and NOT with 'python test_grasping_status_server.py'.

WARNING: These test requires a connection to Robot DE NIRO

Author: John Lingi
Date: 05/18
"""
import rospy
import unittest
import baxter_interface
import sys
import os

root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, 'src/grasping/src'))
from grasping_status_server import GraspingStatusServer, untuck_arms

from grasping.srv import GraspingStatus, GraspingStatusResponse

server = GraspingHandlerServer("grasping_status_service", GraspingStatus)


class GraspingStatusServerTests(unittest.TestCase):

    def test_calibrate(self):
        """
        Unit test for the calibrate function
        :return:
        """

        # True because gripper is a baxter_interface.Gripper object
        gripper = baxter_interface.Gripper('right')
        self.assertTrue(server.calibrate_grippers(gripper))

        # True because gripper is a baxter_interface.Gripper object
        gripper = baxter_interface.Gripper('left')
        self.assertTrue(server.calibrate_grippers(gripper))

        # False because gripper is not a baxter_interface.Gripper object
        gripper = "left"
        self.assertFalse(server.calibrate_grippers(gripper))

    def test_no_param_func_checks(self):
        """
        Unit test for the check_grasping_status function and subordinate functions.
        :return:
        """

        self.assertTrue(server.enable_arms())

        self.assertTrue(untuck_arms())


if __name__ == "__main__":
    rospy.init_node("grasping_status_tests")
    unittest.main()