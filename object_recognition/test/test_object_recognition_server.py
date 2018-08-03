#!/usr/bin/env python
"""
Unit test for the object_recognition_server.py.

NOTE: This should be run via 'rosrun object_recognition test_grasping_handler_server.py' and NOT with 'python test_object_recognition_server.py'.

WARNING: These test requires a connection to Robot DE NIRO

Author: John Lingi
Date: 05/18
"""
import rospy
import unittest
import sys
import os
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point
from numpy import median

root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, 'src/object_recognition/src'))
sys.path.append(os.path.join(root_path, 'src/ros_comms'))

from object_recognition.srv import ObjectRecognition, ObjectRecognitionResponse
from object_recognition_server import ObjectRecognitionServer
from object_recognition_client import ObjectRecognitionClient

server = ObjectRecognitionServer("object_recognition_server", ObjectRecognition)


class ObjectRecognitionServerTests(unittest.TestCase):

    def test_configs(self):
        self.assertEqual(type(server.config), dict)
        self.assertTrue(len(server.config) > 0)

    def test_median_points(self):
        client = ObjectRecognitionClient("object_recognition_server", ObjectRecognition)

        median_points = client.make_request()
        self.assertEqual(type(median_points), Point)

    def test_server_attributes_after_call(self):
        self.assertEqual(len(server.x), server.max_data_points)
        self.assertEqual(len(server.y), server.max_data_points)
        self.assertEqual(len(server.z), server.max_data_points)
        self.assertFalse(server.take_measurements)


if __name__ == "__main__":
    rospy.init_node("object_recognition_tests")
    suite = unittest.TestLoader().loadTestsFromTestCase(ObjectRecognitionServerTests)
    unittest.TextTestRunner(verbosity=2).run(suite)
    rospy.spin()