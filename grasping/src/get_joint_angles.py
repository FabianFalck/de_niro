#!/usr/bin/env python
"""
Short script to get the joint angles of the arm in a given position.

Author: John Lingi
Date: 05/18
"""

import rospy
import baxter_interface

if __name__ == "__main__":
    rospy.init_node("get_joint_angles", log_level=rospy.DEBUG)

    arm_l = baxter_interface.Limb("left")
    arm_r = baxter_interface.Limb("right")
    rospy.loginfo(arm_l.joint_angles())
    rospy.loginfo(arm_r.joint_angles())
