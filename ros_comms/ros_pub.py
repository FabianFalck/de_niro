#!/usr/bin/env python
"""
Wrapper to ROS publisher.

Author: Fabian Falck
Date: 05/18
"""

import rospy


class ROSPublisher(object):

    def __init__(self, _topic, _message_type, _queue_size=1, rate=10):
        """
        ROSPublisher constructor.

        :param _topic: string, ROS topic to publish on
        :param _message_type: custom message, published on topic
        :param queue_size: int, How many messages to queue when publishing. With the default, a subscriber will only take the latest message.
        :param rate: int, how often to publish
        """
        self.topic = _topic
        self.message_type = _message_type
        self.pub = rospy.Publisher(self.topic, self.message_type, queue_size=_queue_size)
        self.rate = rospy.Rate(rate)

    def publish(self, data=None):
        """
        Publishing one message on the initialized topic.

        :param data: any data required for publishing e.g. when having publisher in subscriber callback
        :return:
        """
        rospy.loginfo("Message published on topic %s", self.topic)
