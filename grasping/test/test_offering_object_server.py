#!/usr/bin/env python
"""
Unit test for the offering_object_server.py.

NOTE: This should be run via 'rosrun grasping test_offering_object_server.py' and NOT with 'python test_offering_object_server.py'.

WARNING: These test requires a connection to Robot DE NIRO

Author: John Lingi
Date: 05/18
"""
import rospy
import unittest
import sys
import os
from geometry_msgs.msg import Point
from std_msgs.msg import Empty


root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, 'src/grasping/src'))
from offering_object_server import OfferingObjectServer


server = OfferingObjectServer("offering_object_service", OfferingObject)

class OfferingObjectServerTests(unittest.TestCase):

    def test_offering_object(self):
        """
        Test offering object service
        :return:
        """

        req = Empty()
        rospy.set_param('limb_to_move', 'right')
        self.assertTrue(server.callback(req))

        rospy.set_param('limb_to_move', 'left')
        self.assertTrue(server.callback(req))


if __name__ == "__main__":
    rospy.init_node("offering_object_tests")
    suite = unittest.TestLoader().loadTestsFromTestCase(OfferingObjectServerTests)
    unittest.TextTestRunner(verbosity=2).run(suite)