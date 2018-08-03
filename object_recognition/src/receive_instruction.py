#!/usr/bin/env python
"""
Recognises the ROS marker on an object card and stores the result on the ROS parameter server.
This is used by the marker_position_listener script to get the coordinates of the object with the
matching ROS marker.

Author: John Lingi
Date; 05/18
"""

import rospy
from std_msgs.msg import Int32MultiArray

VALID_TAG_IDS = [102, 104, 101]
REQUESTED_OBJECT = "requested_object"
RECEIVE_INSTRUCTION = "receive_instruction"


def save_object_tag_cb(tag_metadata):
    """
    Callback function for subscriber listening to /object_handler/object_name topic which publishes visible
    ROS marker tags as integer values
    :param tag_metadata: Int32 array of tags
    :return: None
    """

    # Exit if not ready to receive a fetch request
    if rospy.get_param(RECEIVE_INSTRUCTION) == False:
        return

    recognised_tags = tag_metadata.data

    if len(recognised_tags) > 1:
        rospy.loginfo("Sorry, I can't fetch more than one object.")

    elif len(recognised_tags) < 1:
        return

    elif recognised_tags[0] in VALID_TAG_IDS:
        tag = recognised_tags[0]
        rospy.set_param(REQUESTED_OBJECT, tag)
        rospy.set_param(RECEIVE_INSTRUCTION, False)
        rospy.loginfo("Retrieving %s", tag)

    else:
        rospy.loginfo("Sorry, but I can't recognise that tag...brrr.")


def listening_brain_execute():
    """
    Brain starts off the marker recognition, which returns true when marker found
    :return: bool
    """
    rospy.set_param(RECEIVE_INSTRUCTION, True)

    while rospy.get_param(REQUESTED_OBJECT) == "":
        pass
        # Must add a timeout, after which we return false

    return True


if __name__ == "__main__":
    rospy.init_node("receive_instruction")

    # Creates a parameter on the server and sets it to blank
    rospy.set_param(REQUESTED_OBJECT, "")
    rospy.set_param(RECEIVE_INSTRUCTION, False)

    # Listens for object marker
    object_card_subscriber = rospy.Subscriber("object_handler/object_name", Int32MultiArray,
                                              save_object_tag_cb, queue_size=1)
    rospy.spin()