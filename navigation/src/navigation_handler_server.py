#!/usr/bin/env python
"""
Server code for the Navigation Handler service.
This service is responsible for moving DE NIRO to and from the warehouse.

Request: string
Response: uint8

Author: Nico Smuts
Date: 05/18
"""

import rospy
import actionlib
import tf
import math
from navigation.srv import NavigationHandler, NavigationHandlerResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty

import sys
import os
root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_server import ROSServiceServer


_90_DEG_CLOCKWISE = - math.pi / 2
_90_DEG_ANTICLOCKWISE = math.pi / 2
pub_reset_mbed_timeout = 0


class NavigationHandlerServer(ROSServiceServer):

    def goal_coordinates(self, location):
        """
        Returns the x, y and yaw values for the specified location
        :param location: String
        :return: double x, double y, double yaw
        """

        coords = {
                    "warehouse": [2.2, 1.2, _90_DEG_ANTICLOCKWISE],
                    "home": [0, 0, 0]
                  }

        return coords[location][0], coords[location][1], coords[location][2]

    def callback(self, request):
        """
        This function will be used to extract the coordinates from a string
        message. These coordinates will be passed onto the move_base node via
        the action client.

        :param request: string, either 'warehouse' or 'home'
        """

        goal = request.goal # Goal is the string value contained in ROS string msg
        rospy.loginfo("I understand my destination to be %s", goal)

        goal_x, goal_y, yaw = self.goal_coordinates(goal)

        rospy.loginfo("x: {0} \n y: {1} \n yaw: {2}.".format(goal_x, goal_y, yaw))

        result = self.move_base_client(goal_x, goal_y, yaw)

        return NavigationHandlerResponse(result)

    def move_base_client(self, x = 0, y = 0, yaw = 0):
        """
        Client to talk to the action server hosted by the move_base package. This is responsible for navigating the robot
        to the destination

        :param x: target position x
        :param y: target position y
        :param yaw: target position yaw
        :return: result of navigation
        """
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()

        # Information sent to goal is a geomtry message with two components:
        # Header message: contains details of coordinate frame and timestamp
        # Pose message:  Point and quarternion components. Point has 3 componenets
        # (x, y, z) and quarternion has 4 normalised components (x,y,z (axis), angle)

        # Header information
        # This line seems to set the frame which the goal is relevant to
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Pose information
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0

        goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = goal_quaternion[0]
        goal.target_pose.pose.orientation.y = goal_quaternion[1]
        goal.target_pose.pose.orientation.z = goal_quaternion[2]
        goal.target_pose.pose.orientation.w = goal_quaternion[3]

        client.send_goal(goal)

        global pub_reset_mbed_timeout
        pub_reset_mbed_timeout.publish()

        wait = client.wait_for_result()

        if not wait:
            client.cancel_all_goals()
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            rospy.loginfo("%s", client.get_state())
            return client.get_state()


if __name__ == "__main__":
    rospy.init_node("navigation_handler", log_level=rospy.DEBUG)
    navigation_handler_server = NavigationHandlerServer("navigation_handler_service", NavigationHandler)

    global pub_reset_mbed_timeout
    pub_reset_mbed_timeout = rospy.Publisher("reset_mbed_timeout", Empty, queue_size=1)

    rospy.loginfo("The movement handler service is running.")
    rospy.spin()
