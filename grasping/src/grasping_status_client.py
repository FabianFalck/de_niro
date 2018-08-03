#!/usr/bin/env python
"""
Client code to interact with the server on the Grasping Status service.
This service is responsible for checking that the robot is enabled, the arms are untucked and the grippers calibrated.

Request: bool
Response: bool

Author: John Lingi
Date: 05/2018
"""

import rospy
from grasping.srv import GraspingStatus, GraspingStatusResponse

import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_client import ROSServiceClient


class GraspingStatusClient(ROSServiceClient):

    def make_request(self, request):
        """
        Makes request to server

        :param request: Bool
        :return: Bool
        """
        server_response = super(GraspingStatusClient, self).make_request(request)
        if server_response is None:
            rospy.loginfo("%s server response was none", self.service_name)
            return None

        return server_response.is_active


if __name__ == "__main__":
    # Test
    rospy.init_node("grasping_status_service_test")
    client = GraspingStatusClient("grasping_status_service", GraspingStatus)
    result = client.make_request(True)
    rospy.loginfo("Grasping status server result: %s", result)
