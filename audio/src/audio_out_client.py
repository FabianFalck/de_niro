#!/usr/bin/env python
"""
Client code to interact with the server on the Audio Out service.
This service provides text-to-speech functionality for DE NIRO.

Request: string
Response: bool

Author: Nico Smuts
Date: 05/2018
"""


import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))

import rospy
from ros_client import ROSServiceClient
from audio.srv import AudioOut, AudioOutResponse


class AudioOutClient(ROSServiceClient):

    def make_request(self, request=None):
        """
        Makes request to server

        :param request: None
        :return: Boolean
        """
        server_response = super(AudioOutClient, self).make_request(request)
        if server_response is None:
            rospy.loginfo("%s server response was none", self.service_name)
            return None

        return server_response.audio_out


if __name__ == "__main__":
    # Test
    rospy.init_node("audio_out")
    client = AudioOutClient("audio_out_service", AudioOut)
    client.make_request("hello")
    rospy.spin()
