#!/usr/bin/env python

"""
Unit test for the grasping_handler_server.py.

NOTE: This should be run via 'rosrun grasping test_grasping_handler_server.py' and NOT with 'python test_grasping_status_server.py'.

WARNING: These test requires a connection to Robot DE NIRO

Author: John Lingi
Date: 05/18
"""
import rospy
import unittest
import sys
import os
from geometry_msgs.msg import Point

root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, 'src/grasping/src'))
from grasping_handler_server import GraspingHandlerServer


server = GraspingHandlerServer("grasping_handler_service", GraspingHandler)

class GraspingHandlerServerTests(unittest.TestCase):

    def test_transform_request_coordinates(self):
        """
        Test transform request returns right values
        :return:
        """
        test_point = Point(0, 0, 0)
        result = server.transform_request_coordinates(test_point)
        self.assertEqual(result, Point(0.30381, -0.092, 0.82))

    def test_get_intermediate_point(self):
        """
        Test get_intermediate_point function
        :return:
        """
        end_point = Point(0, 0, 0)
        offset = [-0.15, 0, 0]
        int_point = server.get_intermediate_point(end_point, offset)
        self.assertEqual(int_point, Point(-0.15, 0, 0))

        offset = [-0.15, 0.2, 0.1]
        int_point = server.get_intermediate_point(end_point, offset)
        self.assertEqual(int_point, Point(-0.15, 0.2, 0.1))

        end_point = Point(-0.111, 0.2, 0.3)
        offset = [-0.1, 0, -0.3]
        int_point = server.get_intermediate_point(end_point, offset)
        self.assertAlmostEquals(int_point.x, -0.211, delta=1e-6)
        self.assertAlmostEquals(int_point.y, 0.2, delta=1e-6)
        self.assertAlmostEquals(int_point.z, 0, delta=1e-6)


if __name__ == "__main__":
    rospy.init_node("grasping_handler_tests")
    suite = unittest.TestLoader().loadTestsFromTestCase(GraspingHandlerServerTests)
    unittest.TextTestRunner(verbosity=2).run(suite)