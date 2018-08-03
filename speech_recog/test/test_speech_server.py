#!/usr/bin/env python
"""
Unit test for the speech_server.py.

NOTE: This should be run via 'rosrun object_recognition test_speech_server.py' and NOT with 'python test_speech_server.py'.

WARNING: These test requires a connection to Robot DE NIRO

Author: Fabian Falck
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
sys.path.append(os.path.join(root_path, 'src/speech_recog/src'))
sys.path.append(os.path.join(root_path, 'src/ros_comms'))

from speech_recog.srv import SpeechRecognition, SpeechRecognitionResponse
from speech_server import SpeechServer
from speech_client import SpeechClient

server = SpeechServer("speech_recognition_service", SpeechRecognition)


class SpeechRecognitionServerTests(unittest.TestCase):

    def test_configs(self):
        self.assertTrue(server.configs['speech_recognition_API'] in ['sphinx', 'google'])

    def test_recognised_object(self):
        client = SpeechClient("speech_recognition_service", SpeechRecognition)
        recognised_object = client.make_request()
        self.assertTrue(recognised_object is not None)


if __name__ == "__main__":
    rospy.init_node("speech_recognition_tests")
    suite = unittest.TestLoader().loadTestsFromTestCase(SpeechRecognitionServerTests)
    unittest.TextTestRunner(verbosity=2).run(suite)
    rospy.spin()



