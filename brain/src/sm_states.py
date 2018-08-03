#!/usr/bin/env python
"""
Class declarations for all the state machine states.

Author: Sagar Doshi
Date: 05/2018
"""

import sys
import os

root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
sys.path.append(os.path.join(root_path, "src/brain/scripts"))
sys.path.append(os.path.join(root_path, "src/audio/src"))
sys.path.append(os.path.join(root_path, "src/face_recog/src"))
sys.path.append(os.path.join(root_path, "src/grasping/src"))
sys.path.append(os.path.join(root_path, "src/navigation/src"))
sys.path.append(os.path.join(root_path, "src/speech_recog/src"))
sys.path.append(os.path.join(root_path, "src/object_recognition/src"))

import rospy
import smach
import random

# Scripts
from status_check_client import status_checks
from initialise_parameters import intialise_ros_parameters
# import receive_instruction # Used when visual instruction is needed.
# The corresponding line must be uncommented in Listening state below and Speech Recognition code commented out.

# Client classes
from navigation_handler_client import NavigationHandlerClient
from offering_object_client import OfferingObjectClient
from grasping_handler_client import GraspingHandlerClient
from audio_out_client import AudioOutClient
from speech_client import SpeechClient
from face_sub import FaceSubscriber

# Service messages
from audio.srv import AudioOut, AudioOutResponse
from speech_recog.srv import SpeechRecognition, SpeechRecognitionResponse
from grasping.srv import GraspingHandler, GraspingHandlerResponse
from grasping.srv import OfferingObjectHandler, OfferingObjectHandlerResponse
from navigation.srv import NavigationHandler, NavigationHandlerResponse
from face_recog.msg import Name_location


# Sleep time specifications between states
TRANSITION_SLEEP_TIME = 1.5
LISTENING_SLEEP_TIME = 5
NEW_CYCLE_SLEEP_TIME = 10

TRANSITION = "Transitioning to"
GRABBED_OBJECT = "grabbed_object"

REMEMBERED_FACE = "remembered_face"
REMEMBER_NOW = "remember_now"
REMEMBERED_NAME = "remembered_name"

REQUESTED_OBJECT = "requested_object"

INTERLOPER_SEEN = "interloper_seen"

audio_client = AudioOutClient("audio_out_service", AudioOut)

class InitRobot(smach.State):

    """
    Things to be checked in this class:
    1. Relevant audio files
    2. Has calibrated fingers
    3. Kinect camera working
    4. Mbed active
    5. Map exists
    6. Set objects to grasp on param server
    """

    def __init__(self):
        smach.State.__init__(self, outcomes = [TRANSITION])
        self.enter_msg = "Please wait until I initialise"

    def execute(self, smdata=None):
        rospy.logdebug('Executing state InitRobot')
        rospy.loginfo(self.enter_msg + "\n")

        audio_client.make_request(self.enter_msg)
        rospy.sleep(TRANSITION_SLEEP_TIME)

        if not status_checks():
            ignore_status = raw_input("Some status checks failed, do you want to continue? [y / n] ")

            if ignore_status.lower() == 'n':
                rospy.signal_shutdown("Status checks failed!")

        return TRANSITION


class Idling(smach.State):
    """
    Waits in this state for any key
    """

    def __init__(self):
        smach.State.__init__(self, outcomes = [TRANSITION])
        self.enter_msg = "I am idle. Ready to rock and roll."

    def execute(self, smdata=None):
        rospy.logdebug('Executing state Idling')
        rospy.loginfo(self.enter_msg + "\n")
        audio_client.make_request(self.enter_msg)

        # Initialise params on ros server
        intialise_ros_parameters()

        # Waiting for keyboard interrupt
        raw_input("Waiting for any keyboard input from user...")

        return TRANSITION


class Listening(smach.State):
    """
    Listens for a user request
    """

    def __init__(self):
        smach.State.__init__(self, outcomes = [TRANSITION])
        self.enter_msg = "Ready for your command."
        self.fail_msg = "Sorry, I did not understand. Please help me."

    def execute(self, smdata=None):
        rospy.logdebug('Executing state Listening')
        rospy.loginfo(self.enter_msg + "\n")
        audio_client.make_request(self.enter_msg)

        # With markers
        # receive_instruction.listening_brain_execute()

        # With speech recognition
        speech_client = SpeechClient("speech_recognition_service", SpeechRecognition)
        speech_client.make_request()

        audio_client.make_request("Command received. I heard: " + rospy.get_param(REQUESTED_OBJECT))

        rospy.sleep(TRANSITION_SLEEP_TIME)
        return TRANSITION


class RememberingUser(smach.State):
    """
    Views requester, matching him/her with known faces.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes = [TRANSITION])
        self.enter_msg = "Just taking a picture. Smile please!"

    def execute(self, smdata=None):
        rospy.sleep(2.0)
        rospy.logdebug('Executing state RememberingUser')
        rospy.loginfo(self.enter_msg + "\n")

        # audio_client.make_request("Wait. . . Do not go just yet. I have to remember your face.")

        rospy.set_param(REMEMBER_NOW, True)

        while not rospy.get_param(REMEMBERED_FACE):
            # Loop infinitely before state is ended
            # Print face_publisher.remembered_face
            pass

        audio_client.make_request(self.enter_msg)

        rospy.sleep(3.0)

        audio_client.make_request("Thanks, " + rospy.get_param(REMEMBERED_NAME))

        rospy.sleep(TRANSITION_SLEEP_TIME)
        return TRANSITION


class Navigating(smach.State):
    """
    Navigate to warehouse
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=[TRANSITION])
        self.enter_msg = "I am navigating through the room."
        self.fail_msg = "Ah! I have gotten stuck! Please help me."
        self.success_msg = "I have successfully reached the warehouse."

    def execute(self, smdata=None):
        rospy.logdebug('Executing state Navigating')
        rospy.loginfo(self.enter_msg + "\n")

        rospy.sleep(2.0)
        audio_client.make_request(self.enter_msg)


        navigation_handler_client = NavigationHandlerClient("navigation_handler_service", NavigationHandler)
        navigation_handler_client.make_request("warehouse")

        rospy.sleep(TRANSITION_SLEEP_TIME)
        return TRANSITION


class Grasping(smach.State):
    """
    Gets object location and grabs object
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=[TRANSITION])
        self.enter_msg = "I am searching for the "
        self.success_msg = "I managed to pick up the object."
        self.cant_reach = "Sorry, my arms are too short, I could not reach the object."

        # Dynamic fail
        self.fail_msg = "I could not pick up the object. Please help."
        self.fail_1 = "Weird. I failed. Let me try again."
        self.fail_2 = "Cut it out, will you!"
        self.fail_3 = "Stop teasing me!"
        self.dynamic_fails = [self.fail_1, self.fail_2, self.fail_3]

    def execute(self, smdata=None):
        attempts = 0

        rospy.logdebug('Executing state Grasping')
        rospy.loginfo(self.enter_msg + rospy.get_param(REQUESTED_OBJECT))
        audio_client.make_request(self.enter_msg + rospy.get_param(REQUESTED_OBJECT))

        grasping_handler_client = GraspingHandlerClient("grasping_handler_service", GraspingHandler)
        result = grasping_handler_client.make_request()
        rospy.sleep(TRANSITION_SLEEP_TIME)

        # Regardless of whether object is in hand, moves on to Returning
        if result is None:
            return TRANSITION

        rospy.loginfo("grasping_result=%s", str(result))

        if result == 0:
            audio_client.make_request(self.cant_reach)

        elif result == 1:
            audio_client.make_request(self.success_msg)

        elif result == 2:
            while result == 2 and attempts < 5:
                if attempts == 0:
                    audio_client.make_request(self.fail_1)
                else:
                    audio_client.make_request(random.choice(self.dynamic_fails))

                rospy.logdebug("Trying again to grasp. Try #%s", str(attempts))
                result = grasping_handler_client.make_request()
                attempts += 1

        rospy.sleep(TRANSITION_SLEEP_TIME)
        return TRANSITION


class Returning(smach.State):
    """
    Returns to home
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=[TRANSITION])
        self.enter_msg  = "I am returning to where I started."
        self.fail_msg   = "Ah! I have gotten stuck! Please help me."

    def execute(self, smdata=None):
        rospy.logdebug('Executing state Returning')
        rospy.loginfo(self.enter_msg + "\n")
        audio_client.make_request(self.enter_msg)

        navigation_handler_client = NavigationHandlerClient("navigation_handler_service", NavigationHandler)
        navigation_handler_client.make_request("home")

        rospy.sleep(TRANSITION_SLEEP_TIME)
        return TRANSITION


class SeekingUser(smach.State):
    """
    Looks for user that made the request
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=[TRANSITION])
        self.enter_msg = "Trying to find "
        self.fail_msg = "I cannot find the original requester. Please help me."

    def execute(self, smdata=None):
        rospy.logdebug('Executing state SeekingUser')
        rospy.loginfo(self.enter_msg + "\n")
        temp_msg = self.enter_msg + rospy.get_param(REMEMBERED_NAME)
        audio_client.make_request(temp_msg)

        face_receiver = FaceSubscriber("face_recognition/name_location", Name_location)


        warned_interloper = False
        while not face_receiver.correct_face_found: # Loop infinitely
            # Enter below loop only once to avoid constant verbalisations
            if rospy.get_param(INTERLOPER_SEEN) and not warned_interloper:
                audio_client.make_request("Hey! You're not, " + rospy.get_param(REMEMBERED_NAME) + "! Get out of the way, you silly fool!")
                warned_interloper = True
            # print("correct face found: ", face_receiver.correct_face_found)

        rospy.sleep(TRANSITION_SLEEP_TIME)
        return TRANSITION


class OfferingObject(smach.State):
    """
    Offers object back to user if DE NIRO has an object
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=[TRANSITION])
        self.enter_msg  = "I see you "
        self.fail_msg   = "Oh no, "

    def execute(self, smdata=None):
        rospy.logdebug('Executing state OfferingObject')
        rospy.loginfo(self.enter_msg + "\n")

        if rospy.get_param(GRABBED_OBJECT) == True:
            audio_client.make_request(self.enter_msg + rospy.get_param(REMEMBERED_NAME) + "! Here's your " + rospy.get_param(REQUESTED_OBJECT) + ".")
            offering_object_client = OfferingObjectClient("offering_object_service", OfferingObjectHandler)
            offering_object_client.make_request()
        else:
            audio_client.make_request(self.fail_msg + rospy.get_param(REMEMBERED_NAME) + ", I failed miserably to pick up the " + rospy.get_param(REQUESTED_OBJECT) + ". Give me one more chance again... Please!")

        rospy.sleep(NEW_CYCLE_SLEEP_TIME)
        return TRANSITION
