#!/usr/bin/env python
"""
Server code for the Object Recognition service.
This service is responsible for receiving the coordinates of the object.

Request:
Response: geometry_msgs/Point

Author: John Lingi
Date: 05/2018
"""

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point
from numpy import median

import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_server import ROSServiceServer
from object_recognition.srv import ObjectRecognition, ObjectRecognitionResponse


REQUESTED_OBJECT = "requested_object"


class ObjectRecognitionServer(ROSServiceServer):

    def __init__(self, _service_name, _service_type):
        super(ObjectRecognitionServer, self).__init__(_service_name, _service_type)
        self.x = []
        self.y = []
        self.z = []
        self.data_point_counter = 1
        self.max_data_points = 20
        self.take_measurements = False
        self.config = {"apple juice": "tag_102", "coffee": "tag_101", "water": "tag_104"}
        self.sub = rospy.Subscriber('tf', TFMessage, self.gather_datapoints_callback)

    def callback(self, request=None):
        """
        Callback function for service 'object_position_service' to get the position of the requested object

        :param req: Empty
        :return: Median x, y, z coordinates of object
        """

        rospy.loginfo("In object recognition server callback.")

        self.x = []
        self.y = []
        self.z = []
        self.data_point_counter = 1
        self.take_measurements = True

        # Hang while waiting for subscriber to get data points
        while self.data_point_counter <= self.max_data_points:
            pass

        median_point = Point(0, 0, 0)
        median_point.x = median(self.x)
        median_point.y = median(self.y)
        median_point.z = median(self.z)

        self.take_measurements = False
        rospy.loginfo("Object detected at (% s, % s, % s) ", median_point.x, median_point.y, median_point.z)

        return median_point

    def gather_datapoints_callback(self, data):
        """
        Callback function for subscriber listening to 'tf' topic. Retrieves coordinates of requested object.

        :param data: tf object
        :return: None
        """

        if not self.take_measurements:
            return

        # Speech recognition
        if data.transforms[0].child_frame_id == self.config[rospy.get_param(REQUESTED_OBJECT)]:

        # For receive instruction
        # if data.transforms[0].child_frame_id == "tag_" + str(rospy.get_param(REQUESTED_OBJECT)):
            if self.data_point_counter <= self.max_data_points:
                rospy.loginfo("Gathering coordinates for %s: %s / %s",
                              data.transforms[0].child_frame_id, self.data_point_counter, self.max_data_points)

                point = data.transforms[0].transform.translation

                self.x.append(point.x)
                self.y.append(point.y)
                self.z.append(point.z)

                self.data_point_counter += 1


if __name__ == "__main__":
    rospy.init_node("object_recognition")
    object_recognition_server = ObjectRecognitionServer("object_recognition_service", ObjectRecognition)
    rospy.spin()
