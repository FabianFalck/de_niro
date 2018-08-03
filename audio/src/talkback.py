#!/usr/bin/env python
"""
Text-to-speech using Soundplay.

Author: Clara Pouletty
Date: 05/18
"""
import rospy
from std_msgs.msg import String

from sound_play.libsoundplay import SoundClient

from std_msgs.msg import String

class TalkBack:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.wavepath = rospy.get_param("~wavepath", "/home/robin/fezzik-project/src/audio/sounds")

        #Create the sound client object
        self.soundhandle = SoundClient()

        rospy.loginfo("Ready for Alexa broadcast")

        rospy.Subscriber('speech_recognition', String, self.talkback)

    def talkback(self, data):
        """
        Print data to console

        :param data:
        :return:
        """
        rospy.loginfo(rospy.get_caller_id() + " Heard: %s", data.data)

        if data.data == "end":
            rospy.sleep(7)
            self.soundhandle.playWave(self.wavepath + "/DENIRO16.wav")
            rospy.sleep(5)
            self.soundhandle.say("Just kidding, I'll get you one now.", self.voice);

    def cleanup(self):
        rospy.loginfo("Shutting down the talkback node")


if __name__ == "__main__":
    # Test
    rospy.init_node("talkback")
    t = TalkBack()
    s = String("John")
    s.data = "end"
    t.talkback(s)
    t.cleanup()
    rospy.spin()
