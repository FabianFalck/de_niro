#!/usr/bin/env python
"""
Client code to interact with the server on the Object Recognition service.
This service is responsible for receiving the coordinates of the object.

Request:
Response: geometry_msgs/Point

Author: John Lingi
Date: 05/2018
"""

import rospy

import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_client import ROSServiceClient


class ObjectRecognitionClient(ROSServiceClient):

    def make_request(self, request=None):
        """
        Makes request to server.

        :return: Bool indicating offering object status
        """
        server_response = super(ObjectRecognitionClient, self).make_request(request)
        if server_response is None:
            rospy.loginfo("%s server response was none", self.service_name)
            return None

        return server_response.object_location
