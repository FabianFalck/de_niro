#!/usr/bin/env python
"""
Client code to interact with the server on the Grasping Handler service.
This service is responsible for operating DE NIRO's arms when requested to pick up an object.

Request: geometry_msgs/Point
Response: unit8

Author: John Lingi & Kim Rants
Date: 05/2018
"""

import rospy
import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
sys.path.append(os.path.join(root_path, "src/object_recognition/src"))
from ros_client import ROSServiceClient
from object_recognition_client import ObjectRecognitionClient
from object_recognition.srv import ObjectRecognition, ObjectRecognitionResponse
from grasping.srv import GraspingHandler, GraspingHandlerResponse

LIMB_TO_MOVE = "limb_to_move"
GRABBED_OBJECT = "grabbed_object"


class GraspingHandlerClient(ROSServiceClient):

    def limb_move_order(self, object_position):
        """
        Chooses the order of which limbs to use to grab object. This is based on the object's position relative to the
        center of the robot

        :param object_position: geometry_msgs/Point, coordinates of object
        :return: string first_attempt, string second_attempt
        """
        kinect_y_offset = -0.092

        first_attempt = "right"
        second_attempt = "left"
        if (-object_position.x + kinect_y_offset) > 0:
            first_attempt = "left"
            second_attempt = "right"

        return first_attempt, second_attempt

    def make_request(self, request=None):
        """
        Request to grasping server. The goal should be empty when called as within this class,
        object recognition is called

        :param request: std_msgs/Empty
        :return:
        """
        rospy.loginfo("Asking object recognition service for object location")

        object_recog_client = ObjectRecognitionClient("object_recognition_service", ObjectRecognition)

        object_position = object_recog_client.make_request()  # Returns object location in Kinect's frame of reference

        if object_position is None:
            return None

        first_attempt, second_attempt = self.limb_move_order(object_position)

        rospy.loginfo("Passing object location to grasping handler server.")

        rospy.loginfo("Limb moving (first attempt): %s ", first_attempt)
        rospy.set_param(LIMB_TO_MOVE, first_attempt)  # This will be called in the handler server

        # Call grasping handler server
        server_response = self._call_service(object_position)
        if server_response is None:
            rospy.loginfo("%s server response was none", self.service_name)
            return None

        grasping_result = server_response.grasping_status
        rospy.loginfo("Result status from grasping: %s", grasping_result)

        # In case of blind spot
        if grasping_result == 0:
            rospy.loginfo("Limb moving (second attempt): %s ", second_attempt)
            rospy.set_param(LIMB_TO_MOVE, second_attempt)

            server_response = self._call_service(object_position)
            if server_response is None:
                rospy.loginfo("%s server response was none", self.service_name)
                return None

            grasping_result = server_response.grasping_status
            rospy.loginfo("Result status from grasping: %s", grasping_result)

        if grasping_result == 1:
            rospy.set_param(GRABBED_OBJECT, True)
        else:
            rospy.set_param(GRABBED_OBJECT, False)

        return grasping_result


if __name__ == "__main__":
    # Test
    rospy.init_node("grasping_handler_service_test")

    rospy.set_param("requested_object", "101")
    rospy.set_param(LIMB_TO_MOVE, "right")
    rospy.set_param(GRABBED_OBJECT, False)

    client = GraspingHandlerClient("grasping_handler_service", GraspingHandler)
    result = client.make_request()
    rospy.loginfo("Grasping handler server result: %s", result)
