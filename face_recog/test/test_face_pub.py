#!/usr/bin/env python
"""
Unit tests for face_pub.py.

NOTE: This should be run via 'rosrun audio test_face_pub.py' and NOT with 'python test_face_pub.py'.

WARNING: These test requires a connection to Robot DE NIRO

Author: Fabian Falck
Date: 05/18
"""

import unittest
import rospy
import sys
import os

root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, "src/face_recog/src"))

from face_pub import FacePublisher
from face_recog.msg import Name_location


class FacePublisherTests(unittest.TestCase):

    def test_instance_variables(self):
        face_publisher = FacePublisher("face_recognition/name_location", Name_location)
        self.assertTrue(len(face_publisher.train_data_names) > 0)


if __name__ == "__main__":
    rospy.init_node("face_publisher_tests")
    suite = unittest.TestLoader().loadTestsFromTestCase(FacePublisherTests)
    unittest.TextTestRunner(verbosity=2).run(suite)
    rospy.spin()