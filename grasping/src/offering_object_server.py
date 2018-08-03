#!/usr/bin/env python
"""
Server code for the Offering Object service.
This service is responsible for operating DE NIRO's arms when requested to pick up an object.

Request:
Response: bool

Author: John Lingi & Kim Rants
Date: 05/2018
"""

import rospy
from grasping.srv import OfferingObjectHandler, OfferingObjectHandlerResponse
from math import fabs
import baxter_interface
from grasping_handler_server import openclose_gripper, return_to_untucked

import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_server import ROSServiceServer


LIMB_TO_MOVE = "limb_to_move"
SLEEP_TIME = 1.0


class OfferingObjectServer(ROSServiceServer):

    def move_to_handover(self, baxter_arm, limb):
        """
        Moves specified limb to the handover position.

        :param baxter_arm: arm to move
        :param limb: which arm, 'right' or 'left
        :return:
        """

        rospy.loginfo("Moving %s to handover position." % limb)
        if limb == "left":
            handover_position_joints = {"left_w0": 0.47016511148687934, "left_w1": -0.42798063982003043,
                                          "left_w2": 0.18676216092504913, "left_e0": -1.1616069516262295,
                                          "left_e1": 1.889097340280887, "left_s0": 0.1499466220157992,
                                          "left_s1": -0.9115680832009071}

            baxter_arm.move_to_joint_positions(handover_position_joints)

        elif limb == "right":
            handover_position_joints = {"right_s0": -0.3282718886074785, "right_s1": -1.117505003974524,
                                           "right_w0": -0.40535442319872056, "right_w1": -0.5042961840173298,
                                           "right_w2": -0.08053399136398422, "right_e0": 1.0346700414287116,
                                           "right_e1": 1.9539080285690458}

            baxter_arm.move_to_joint_positions(handover_position_joints)
        else:
            rospy.logerr("Bad input to function move_to_handover. %s is not a valid limb parameter" % limb)

    def callback(self, request=None):
        """
        Call back for handover service.

        :param request: std_msgs/Empty
        :return: True
        """
        limb = rospy.get_param(LIMB_TO_MOVE)
        arm = baxter_interface.Limb(limb)
        gripper = baxter_interface.Gripper(limb)

        self.move_to_handover(arm, limb)
        rospy.sleep(SLEEP_TIME)

        end_effector_position_initial = arm.endpoint_pose()["position"]
        end_effector_position_current = arm.endpoint_pose()["position"]

        offset = 0.02
        while fabs(end_effector_position_current.z - end_effector_position_initial.z) < offset:
            end_effector_position_current = arm.endpoint_pose()["position"]

        rospy.loginfo("Opening my gripper")
        openclose_gripper(gripper, cmd='open')
        rospy.sleep(0.5)

        return_to_untucked(arm, limb)

        return True


if __name__ == "__main__":
    rospy.init_node("offering_object")
    offering_object_server = OfferingObjectServer("offering_object_service", OfferingObjectHandler)
    rospy.loginfo("The offering object server is running.")
    rospy.spin()
