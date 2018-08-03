#!/usr/bin/env python
"""
File to test the state machine

Author: Sagar Doshi
Date: 05/2018
"""

import unittest, sys, os

root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, 'src/brain/src'))
from sm_states import InitRobot, Listening, Navigating, Grasping, Returning, SeekingUser, OfferingObject


TRANSITION = 'Transitioning to'
class SM_Tests(unittest.TestCase):

    def setUp(self):
        pass

    def test_doubler(self):
        # Just to ensure that the testing itself works
        # Quis custodiet ipsos custodes?
        self.assertEqual(doubler(-48), -96)

    def test_StateMessages(self):
        initialiser = InitRobot()
        self.assertEqual(initialiser.enter_msg, "Please wait until I initialise")
        self.assertEqual(initialiser.fail_msg, "Sorry, I think I failed to initialise. Please help me.")

        listener = Listening()
        self.assertEqual(listener.enter_msg, "I am ready for your command.")
        self.assertEqual(listener.fail_msg, "Sorry, I think I did not understand. Please help me.")

        navigator = Navigating()
        self.assertEqual(navigator.enter_msg, "I am navigating my way through the room.")
        self.assertEqual(navigator.fail_msg, "Ah! I have gotten stuck! Please help me.")

        grasper = Grasping()
        self.assertEqual(grasper.enter_msg, "I have found the object and am picking it up.")
        self.assertEqual(grasper.fail_msg, "I could not pick up the object. Please help me.")

        returner = Returning()
        self.assertEqual(returner.enter_msg, "I am returning to where I started.")
        self.assertEqual(returner.fail_msg, "Ah! I have gotten stuck! Please help me.")

        seeker = SeekingUser()
        self.assertEqual(seeker.enter_msg, "I am trying to find the person who made the initial request.")
        self.assertEqual(seeker.fail_msg, "I cannot find the original requester. Please help me.")

        offerer = OfferingObject()
        self.assertEqual(offerer.enter_msg, "I am trying to give away the object I found.")
        self.assertEqual(offerer.fail_msg, "I could not give away the object I found. Please help me.")

    def test_executes(self):
        initialiser = InitRobot()
        self.assertEqual(initialiser.execute(TRANSITION))

        listener = Listening()
        self.assertEqual(listener.execute(TRANSITION))

        navigator = Navigating()
        self.assertEqual(navigator.execute(TRANSITION))

        grasper = Grasping()
        self.assertEqual(grasper.execute(TRANSITION))

        returner = Returning()
        self.assertEqual(returner.execute(TRANSITION))

        seeker = SeekingUser()
        self.assertEqual(seeker.execute(TRANSITION))

        offerer = OfferingObject()
        self.assertEqual(offerer.execute(TRANSITION))


def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(SM_Tests, 'test'))
    return suite


if __name__ == '__main__':
    unittest.main()
