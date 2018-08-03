#!/usr/bin/env python
"""
Face recognition publisher to recognise face from Kinect video stream and store it in the 'remembered_face' ROS parameter.

general source: https://github.com/ageitgey/face_recognition/blob/master/README.md
code source 1: https://github.com/ageitgey/face_recognition/blob/master/examples/facerec_from_webcam_faster.py
code source 2: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

Author: Fabian Falck
Date: 05/18
"""

from __future__ import print_function
import face_recognition
import os
import sys
from face_recog.msg import Name_location

# Imports required for OpenCV Bridge processing
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

REMEMBERED_NAME = "remembered_name"
REMEMBERED_FACE = "remembered_face"
REMEMBER_NOW = "remember_now"

root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_pub import ROSPublisher


class FacePublisher(ROSPublisher):

    def __init__(self, _topic, _message_type, queue_size=1, rate=10):
        super(FacePublisher, self).__init__(_topic, _message_type, queue_size, rate)

        self.image_database_path = root_path + "/src/face_recog/src"  # If changed, must be changed in face_client, too

        # Train data and meta data names
        self.train_data_names = [("Fabian.jpg", "Fabian"), ("Kim.jpg", "Kim"), ("John.jpg", "John"), ("Nico.jpg", "Nico"), ("Sagar.jpg", "Sagar")]
        # Join the image names with the absolute path of where the images are above
        self.train_data_names = [(os.path.join(self.image_database_path, filename), name_of_person) for (filename, name_of_person) in self.train_data_names]

        # Load a sample picture and learn how to recognize it.
        self.images = [face_recognition.load_image_file(train_data_tup[0]) for train_data_tup in self.train_data_names]

        # there can be more than one face in one image, that is why there might be multiple matches
        self.train_face_encodings = [face_recognition.face_encodings(image)[0] for image in self.images]

        # Initialize some variables
        self.face_locations = []
        self.face_names = []
        self.process_this_frame = True

        # Whether face is remembered or not
        self.remembered_face = False

        # CVBridge is required to transform the ros msg data, as published by the kinect drivers,
        # into OpenCV images for further processing
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/hd/image_color",Image,self.cv_bridge_callback)

        rospy.loginfo("FacePublisher started...")
        # Helping comment: You can test whether publishing works on terminal: 'rostopic echo /face_recognition/name_location'

    def cv_bridge_callback(self, data):
        """
        Callback processing a single frame whenever a new image message from the Kinect server appears

        :param data: Ros message with type sensor_msgs.msg/Image
        :return:
        """

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        rows, cols, channels = cv_image.shape
        if cols > 60 and rows > 60 :
          cv2.circle(cv_image, (50,50), 10, 255)

        # cv2.imshow("CVBridge processing", cv_image)
        cv2.waitKey(3)

        self.publish(cv_image)

    def publish(self, frame):
        """
        Publishing the recognized faces in the video frames

        :param frame:
        :return:
        """

        # Resize frame of video to 1/4 size for faster facce recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        # Only process every other frame of video to save time
        if self.process_this_frame:
            # Find all the faces and face encodings in the current frame of video
            self.face_locations = face_recognition.face_locations(small_frame)
            test_face_encodings = face_recognition.face_encodings(small_frame, self.face_locations)

            l = 0
            self.face_names = []
            for face_encoding in test_face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(self.train_face_encodings, face_encoding)

                # "Normal case": Find any face encodings in the
                # compare the test face encodings with the the train ones (the ones we are looking for)
                # assign it the correct name
                name = "Unknown"  # default name if no match is found
                for i, match in enumerate(matches):
                    if (match):
                        name = self.train_data_names[i][1]
                        if len(self.face_locations) > l:  # TODO why can face_locations ever not have values when there is a match?
                            pass
                        l += 1
                self.face_names.append(name)

            # Data transformation of self.face_locations so to publish it: from list of tuples to list (due to data type of custom message)
            face_locations_transform = []
            for loc in self.face_locations:
                for i in range(0,4):  #  4 values per location tuple
                    face_locations_transform.append(loc[i])

            # Publish the faces and their corresponding locations (tuples of 4 long values in a custom message)
            # The custom message consists of two arrays (string and int64 array)
            msg = Name_location()
            msg.names = self.face_names
            msg.locations = face_locations_transform
            self.pub.publish(msg)

            # "Special case": When remember_first is set, the first time only one face is recognized in the image, the name corresponding
            # to the face in that image is stored in a text file so to later know which person was initially recognized
            if rospy.get_param(REMEMBER_NOW) and len(self.face_names)==1:
                name = self.face_names[0]

                # Storing the remembered face on the ROS parameter server
                rospy.loginfo("Face name remembered: " + name)
                rospy.set_param(REMEMBERED_NAME, name)

                rospy.set_param(REMEMBER_NOW, False)  # face recognized, will now be stored
                self.remembered_face = True

                rospy.set_param(REMEMBERED_FACE, True)

        self.process_this_frame = not self.process_this_frame

        # Display the results
        for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), -1)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        cv2.imshow("Face recognition", small_frame)


if __name__ == "__main__":
    rospy.init_node("face_recog")
    face_publisher = FacePublisher("face_recognition/name_location", Name_location)
    rospy.spin()
