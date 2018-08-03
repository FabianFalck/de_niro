#!/usr/bin/env python
"""
Face recognition subscriber to receive face from 'remembered_name' ROS parameter and compare the currently detected face
with the remembered face.

Author: Fabian Falck
Date: 05/18
"""

import rospy
from face_recog.msg import Name_location
from face_pub import REMEMBERED_NAME

import sys
import os

INTERLOPER_SEEN = "interloper_seen"

root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_sub import ROSSubscriber
from face_pub import FacePublisher

class FaceSubscriber(ROSSubscriber):

    def __init__(self, _topic, _message_type):
        super(FaceSubscriber, self).__init__(_topic, _message_type)
        self.correct_face_found = False

        # Read the name of the person that shall be fond once
        self.remembered_name = rospy.get_param(REMEMBERED_NAME)

    def callback(self, msg):
        """
        Information of the custom message is available as instance variables,
        e.g. msg.names, msg.locations

        :param msg: custom message Name_location.msg
        :return: None
        """

        super(FaceSubscriber, self).callback()
        if self.remembered_name in msg.names:
            rospy.loginfo("Found correct face!")
            self.correct_face_found = True
        # Adding else case if we've seen someone who is NOT the requester
        elif (msg.names != []) and (rospy.get_param(INTERLOPER_SEEN) == False):
            rospy.set_param(INTERLOPER_SEEN, True)
            rospy.loginfo("Seen incorrect face.")


if __name__ == "__main__":
    # Test
    rospy.init_node("face_recog")
    face_subscriber = FaceSubscriber("face_recognition/name_location", Name_location)

    # For coverage tests:
    face_publisher = FacePublisher('face_recognition/name_location', Name_location)
    rospy.spin()
