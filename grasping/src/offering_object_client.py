#!/usr/bin/env python
"""
Client code to interact with the server on the Offering Object service.
This service is responsible for operating DE NIRO's arms when requested to pick up an object.

Request:
Response: bool

Author: John Lingi
Date: 05/2018
"""

import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))

import rospy
from ros_client import ROSServiceClient


class OfferingObjectClient(ROSServiceClient):

    def make_request(self, request=None):
        """
        Makes request to server.

        :return: Bool indicating offering object status
        """
        server_response = super(OfferingObjectClient, self).make_request()
        if server_response is None:
            rospy.loginfo("%s server response was none", self.service_name)
            return None

        return server_response.offering_object_status
