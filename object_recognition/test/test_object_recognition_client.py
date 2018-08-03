#!/usr/bin/env python
"""
Client code to test the Object recognition client

Author: John Lingi
Date: 05/2018
"""


import sys
import os
root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, 'src/object_recognition/src'))
sys.path.append(os.path.join(root_path, 'src/ros_comms'))

import rospy
from object_recognition_client import ObjectRecognitionClient
from object_recognition_server import ObjectRecognitionServer
from object_recognition.srv import ObjectRecognition, ObjectRecognitionResponse


if __name__ == "__main__":
    rospy.init_node("object_recognition_client_test")
    rospy.set_param('requested_object', "101")

    server = ObjectRecognitionServer("object_recognition_service", ObjectRecognition)
    client = ObjectRecognitionClient('object_recognition_service', ObjectRecognition)
    print(client.make_request())
    rospy.spin()