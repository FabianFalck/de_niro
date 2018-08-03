#!/usr/bin/env python
"""
Wrapper to ROS subscriber.

Author: Fabian Falck
Date: 05/18
"""

import rospy


class ROSSubscriber(object):

    def __init__(self, _topic, _message_type):
        """
        ROSSubscriber constructor.

        :param _topic: string, ROS topic to publish on
        :param _message_type: custom message, published on topic
        """
        self.topic = _topic
        self.message_type = _message_type
        self.sub = rospy.Subscriber(self.topic, self.message_type, self.callback)

    def callback(self, data=None):
        """
        Called when new message arrives on topic.
        :param data: Data on topic
        :return:
        """
        rospy.loginfo("Received message on topic %s", self.topic)

