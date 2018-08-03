#!/usr/bin/env python
"""
Unit tests for the audio_out_server.py.

NOTE: These tests require the audio_out_server to be running 'rosrun audio audio_out_server.py'.
NOTE: This should be run via 'rosrun audio test_audio_out_server.py' and NOT with 'python test_audio_out_server.py'.

WARNING: These test requires a connection to Robot DE NIRO

Author: Nico Smuts
Date: 05/18
"""
import rospy
import unittest
import time

import sys, os
root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, "src/audio/src"))
from audio_out_client import AudioOutClient


class AudioOutServerTests(unittest.TestCase):

    def test_successful_pronounce(self):
        client = AudioOutClient()
        self.assertTrue(client.pronounce("hello"))

    def test_wrong_input(self):
        client = AudioOutClient()
        self.assertRaises(Exception, client.pronounce(2))

    def test_pronounce_sequence(self):
        client = AudioOutClient()
        t1 = time.time()
        client.pronounce("I am idle. Tap a key on me to start.")
        t2 = time.time()
        client.pronounce("I am ready for your command.")
        t3 = time.time()
        client.pronounce("Sorry, I think I did not understand. Please help me.")

        self.assertGreater(t2, t1)
        self.assertGreater(t3, t2)


if __name__ == "__main__":
    rospy.init_node("audio_server_tests", anonymous=True)
    suite = unittest.TestLoader().loadTestsFromTestCase(AudioOutServerTests)
    unittest.TextTestRunner(verbosity=2).run(suite)