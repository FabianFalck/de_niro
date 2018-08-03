"""
ROS service base class

Author: John Lingi
Date: 05/18
"""

import rospy


class ROSServiceServer(object):
    """
    NOTE: A node should not be initialised within an object. To run the server, you will need to have initialised a node first.
    """
    def __init__(self, _service_name, _service_type):
        """
        Server constructor.

        :param _service_name: string, service name
        :param _service_type: custom type for server request and response
        """
        self.service_name = _service_name
        self.service_type = _service_type
        self.server = rospy.Service(self.service_name, self.service_type, self.callback)
        rospy.loginfo("Server instantiated: %s", self.service_name)

    def callback(self, request):
        """
        Server callback function to respond to request

        :param request:
        :return:
        """
        rospy.loginfo("%s received request", self.service_name)


