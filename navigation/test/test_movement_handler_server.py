#!/usr/bin/env python
"""
Unit test for the navigation_handler_server.py.

NOTE: This should be run via 'rosrun navigation test_navigation_handler_server.py' and NOT with 'python test_navigation_handler_server.py'.

WARNING: These test requires a connection to Robot DE NIRO

Author: Nico Smuts
Date: 05/18
"""
import rospy
import unittest
import sys
import os
import math


root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, 'src/navigation/src'))
from navigation_handler_server import NavigationHandlerServer
from navigation.srv import NavigationHandler, NavigationHandlerResponse


server = NavigationHandlerServer("navigation_handler_service", NavigationHandler)


class NavigationHandlerServerTests(unittest.TestCase):

    def test_goal_coordinates(self):
        location = 'warehouse'
        x, y, yaw = server.goal_coordinates(location)
        self.assertEqual(x, -0.85)
        self.assertEqual(y, 1.4)
        self.assertEqual(yaw, math.pi / 2)

        location = 'home'
        x, y, yaw = server.goal_coordinates(location)
        self.assertEqual(x, 0)
        self.assertEqual(y, 0)
        self.assertEqual(yaw, 0)


if __name__ == "__main__":
    rospy.init_node("grasping_handler_tests")
    suite = unittest.TestLoader().loadTestsFromTestCase(NavigationHandlerServerTests)
    unittest.TextTestRunner(verbosity=2).run(suite)
    rospy.spin()
