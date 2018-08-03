#!/usr/bin/env python
"""
Runs status checks for grasping component when DE NIRO is first launched. This is to check
that these components are functioning properly.

Author: Sagar Doshi
Date: 05/2018
"""

import sys, os
import rospy
from grasping.srv import GraspingStatus, GraspingStatusResponse

root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/grasping/src"))

from grasping_status_client import GraspingStatusClient


def status_checks():
    """
    Runs status checks for the grasping

    :return: None. Exception will be raised if something fails
    """

    feature_name = "grasping"
    rospy.loginfo("Requesting %s's status.", feature_name)
    grasping_status_client = GraspingStatusClient("grasping_status_service", GraspingStatus)
    response = grasping_status_client.make_request(True)
    if response is None or not response:
        rospy.logerr("%s's status checks failed.", feature_name)
        return False

    else:
        rospy.loginfo("%s's status checks were successful.", feature_name)
        return True


if __name__ == "__main__":
    rospy.init_node("brain_status_check")
    rospy.loginfo(status_checks())
