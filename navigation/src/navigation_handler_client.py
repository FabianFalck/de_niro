#!/usr/bin/env python
"""
Client code to interact with the server on the Navigation Handler service.
This service is responsible for moving DE NIRO to and from the warehouse.

Request: string
Response: uint8

Author: John Lingi
Date: 05/2018
"""

import sys
import os
root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))

import rospy
from navigation.srv import NavigationHandler, NavigationHandlerResponse
from ros_client import ROSServiceClient


class NavigationHandlerClient(ROSServiceClient):

    def make_request(self, request):
        """
        Makes request to server.

        :param request: None
        :return: bool
        """
        server_response = super(NavigationHandlerClient, self).make_request(request)
        if server_response is None:
            rospy.loginfo("%s server response was none", self.service_name)
            return None

        return server_response.navigation_status


if __name__ == "__main__":
    # Test - require navigation to be on
    rospy.init_node("navigation_handler_test")
    goal = "home"
    navigation_client = NavigationHandlerClient('navigation_handler_service', NavigationHandler)
    print(navigation_client.make_request(goal))
