#!/usr/bin/env python
"""
Initialises parameters used by other components, on the ROS parameter server.

Author: Sagar Doshi
Date: 05/2018
"""

import rospy

import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
sys.path.append(os.path.join(root_path, "src/brain/scripts"))
sys.path.append(os.path.join(root_path, "src/audio/src"))
sys.path.append(os.path.join(root_path, "src/face_recog/src"))
sys.path.append(os.path.join(root_path, "src/grasping/src"))
sys.path.append(os.path.join(root_path, "src/navigation/src"))
sys.path.append(os.path.join(root_path, "src/speech_recog/src"))
sys.path.append(os.path.join(root_path, "src/object_recognition/src"))

LIMB_TO_MOVE = "limb_to_move"
GRABBED_OBJECT = "grabbed_object"
REQUESTED_OBJECT = "requested_object"
RECEIVE_INSTRUCTION = "receive_instruction"
REMEMBERED_FACE = "remembered_face"
REMEMBERED_NAME = "remembered_name"
REMEMBER_NOW = "remember_now"
INTERLOPER_SEEN = "interloper_seen"
LEFT_ARM_NEUTRAL = "left_arm_neutral"
RIGHT_ARM_NEUTRAL = "right_arm_neutral"


def intialise_ros_parameters():
    """
    Function called by the brain to initialise all parameters on the ROS parameter server

    :return:
    """
    rospy.set_param(LIMB_TO_MOVE, "right")
    rospy.set_param(GRABBED_OBJECT, False)
    rospy.set_param(REQUESTED_OBJECT, "")
    rospy.set_param(RECEIVE_INSTRUCTION, False)
    rospy.set_param(REMEMBERED_FACE, False)
    rospy.set_param(REMEMBER_NOW, False)
    rospy.set_param(REMEMBERED_NAME, "")
    rospy.set_param(INTERLOPER_SEEN, False)

    left_arm_neutral = {'left_w0': 0.5982525072753113, 'left_w1': 0.6688156235180404,
    'left_w2': -0.49931074645670215, 'left_e0': -1.5159565136277602, 'left_e1': 2.1663643676911755,
    'left_s0': 0.6864564025787226, 'left_s1': -1.0339030510347689}

    right_arm_neutral = {'right_s0': -0.4866554049566475, 'right_s1': -1.1792477306869118,
    'right_w0': -0.016106798272796843, 'right_w1': 0.7125340759727747, 'right_w2': -0.4387185053352283,
    'right_e0': 1.2459758949599273, 'right_e1': 2.12801484799404}

    rospy.set_param(LEFT_ARM_NEUTRAL, left_arm_neutral)
    rospy.set_param(RIGHT_ARM_NEUTRAL, right_arm_neutral)



if __name__ == "__main__":
    rospy.init_node("set_parameters")
    intialise_ros_parameters()
