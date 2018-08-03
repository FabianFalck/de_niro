"""
Client code to interact with the server on the Speech Recognition service.
This service is responsible for recognising speech.

Request:
Response: string

Author: Fabian Falck
Date: 05/18
"""

import rospy
import sys, os

root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_client import ROSServiceClient

REQUESTED_OBJECT = "requested_object"

class SpeechClient(ROSServiceClient):

    def make_request(self, request=None):
        """
        Makes request to server

        :return: String
        """
        server_response = super(SpeechClient, self).make_request(request)
        if server_response is None:
            rospy.loginfo("%s server response was none", self.service_name)
            return None

        rospy.set_param(REQUESTED_OBJECT, server_response.recognised_object)
        return server_response.recognised_object
