#!/usr/bin/env python
"""
Recognises markers in Kinect image stream and publishes data associated with each tag e.g. tag id, location.

Author: Fabian Falck
Date: 05/18
"""

import sys
import os

import rospy
from std_msgs.msg import Int32MultiArray
import tf

root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))

from ros_pub import ROSPublisher


class DetectedMarkersPublisher(ROSPublisher):

    def __init__(self, _topic, _message_type, queue_size=1, rate=10):
        super(DetectedMarkersPublisher, self).__init__(_topic, _message_type, queue_size, rate)

        self.listener = tf.TransformListener(True, rospy.Duration(1.0))

        self.used_tag_ids = range(100, 105)  # Must correspond to tag_id_to_object dictionary in marker_subscriber

        self.cam_frame = "kinect2_rgb_optical_frame"

    def publish(self, data=None):
        super(DetectedMarkersPublisher, self).publish(self)

        frame_strings = self.listener.getFrameStrings()
        cur_time = rospy.get_rostime()
        frame_strings = [x for x in frame_strings if x[0:4] == "tag_"]  #
        frame_strings = [int(x[4:]) for x in frame_strings if (cur_time-self.listener.getLatestCommonTime(x, self.cam_frame))<rospy.Duration(.3)]

        # Object_ids = [tag_id[4:] for tag_id in frame_strings if tag_id in tag_id_to_object]
        objects = [tag_id for tag_id in frame_strings if tag_id in self.used_tag_ids]  # Required, since otherwise a lot of noise (various other tags) are recognized
        objects = set(objects)

        if len(objects) == 0:
            rospy.loginfo("No object tag recognized!")

        elif len(objects) > 0:
            out_str = ""
            for obj in objects:
                out_str += str(obj) + ', '
            # Remove comma from last object
            if out_str[-2:] == ', ':
                out_str = out_str[:-2]

            rospy.loginfo("Objects recognised %s", out_str)

        # Prepare the message
        msg = Int32MultiArray(data=objects)  # std_msgs.msg.UInt8STH()

        # Publish the message
        self.pub.publish(msg)

        # Make control sleep according to the specified rate above
        self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("object_handler")
    pub = DetectedMarkersPublisher("object_handler/object_name", Int32MultiArray)
    while not rospy.is_shutdown():
        pub.publish()
