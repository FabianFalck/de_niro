#!/usr/bin/env python
"""
A custom client class to interact with a ROS service.

Author: John Lingi
Date: 05/18
"""

import rospy


class ROSServiceClient(object):
    def __init__(self, _service_name, _service_type):
        """
        Client constructor.

        :param _service_name: string, service name
        :param _service_type: custom type for server request and response
        """
        self.service_name = _service_name
        self.service_type = _service_type
        self.service_proxy = rospy.ServiceProxy(self.service_name, self.service_type)
        rospy.loginfo("Client instantiated: %s", self.service_name)

    def _call_service(self, request=None, time_limit=5):
        """
        Calls the service with a specified time limit. This returns the entire response not filtered for attributes.

        See NavigationClient for an example.

        :param request: service request
        :return: service response
        """
        rospy.loginfo("Waiting for %s for a maximum of %s seconds.", self.service_name, time_limit)

        try:
            rospy.wait_for_service(self.service_name, time_limit)
            if request is None:
                server_response = self.service_proxy()
            else:
                server_response = self.service_proxy(request)
            return server_response

        except rospy.exceptions.ROSException, e:
            rospy.logerr("Service call failed: %s", e)
            return None

    def make_request(self, request=None):
        """
        Client to interact with ROS service server.

        :param request:
        :return:
        """

        return self._call_service(request)
