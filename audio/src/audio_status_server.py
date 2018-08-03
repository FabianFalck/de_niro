#!/usr/bin/env python
"""
Server code for the Audio Out status service.
This service checks if the necessary audio files exist.

Request: bool
Response: bool

Author: Clara Pouletty
Date: 05/18
"""

import rospy
import os

import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, 'src/ros_comms'))

from audio.srv import AudioStatus, AudioStatusResponse
from ros_server import ROSServiceServer

voice = "voice_kal_diphone"
wavepath = "/home/robin/fezzik-project/src/audio/sounds"

class AudioStatusServer(ROSServiceServer):

    def do_audio_o_files_exist(self):
        """
        Helper function to verify if correct files are in place
        :return: Boolean
        """
        wav_exists = os.path.exists(wavepath + "/DENIRO16.wav")
        kal_diphone_filepath = "/usr/share/festival/voices/english/kal_diphone/festvox/kal_diphone.scm"
        voice_exists = os.path.exists(kal_diphone_filepath)

        if voice_exists and wav_exists:
            return True
        else:
            return False

    def set_audio_o_params(self):
        """
        Helper function to abstract parameter settings
        :return:
        """
        rospy.set_param("~voice", voice)
        rospy.set_param("~wavepath", wavepath)

    def callback(self, request=None):
        """
        Printout function based on whether audio files in place
        :param request: Empty
        :return: Boolean
        """
        if self.do_audio_o_files_exist():
            rospy.loginfo("All audio files in place for Audio Out.")
            return True
        else:
            rospy.logerr("The correct files are not in place for Audio Out.")
            return False


if __name__ == "__main__":
    rospy.init_node("audio_status")

    server = AudioStatusServer("audio_status_service", AudioStatus)
    server.do_audio_o_files_exist()
    server.set_audio_o_params()
    server.callback("")

    rospy.logdebug("The Audio status server is running.")
    rospy.spin()
