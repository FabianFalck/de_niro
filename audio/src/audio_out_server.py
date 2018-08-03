#!/usr/bin/env python
"""
Server code for the Audio Out service.
This service provides text-to-speech functionality for DE NIRO.

Request: string
Response: bool

Author: Nico Smuts
Date: 05/18
"""

import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))

import rospy
from espeak import espeak
from audio.srv import AudioOut, AudioOutResponse
from ros_server import ROSServiceServer


espeak.set_voice("en-fezzik")


class AudioOutServer(ROSServiceServer):

    def callback(self, request):
        """
        Passes string to espeak to pronounce

        :param request: std_msgs/String
        :return: Bool
        """
        phrase = request.phrase
        espeak.synth(phrase)
        return AudioOutResponse(True)


if __name__ == "__main__":
    rospy.init_node("audio_out")
    server = AudioOutServer("audio_out_service", AudioOut)
    rospy.spin()
