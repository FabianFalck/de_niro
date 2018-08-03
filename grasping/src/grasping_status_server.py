#!/usr/bin/env python
"""
Server code for the Grasping Status service.
This service is responsible for checking that the robot is enabled, the arms are untucked and the grippers calibrated.

Request: bool
Response: bool

Author: John Lingi & Kim Rants
Date: 05/2018
"""

import rospy
from grasping.srv import GraspingStatus, GraspingStatusResponse
from baxter_interface import Gripper, RobotEnable, Limb, CHECK_VERSION
import tuck_arms

import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_server import ROSServiceServer

LEFT_ARM_NEUTRAL = "left_arm_neutral"
RIGHT_ARM_NEUTRAL = "right_arm_neutral"

class GraspingStatusServer(ROSServiceServer):

    def calibrate_grippers(self, gripper):
        """
        Calibrates the grippers if not already calibrated.

        :param gripper: Baxter_interface.Gripper object
        :return: bool to indicate if calibration was successful or not
        """
        sleep_time = 1

        try:
            name = gripper.name

            if not gripper.calibrated():
                rospy.loginfo("Calibrating %s." % name)
                gripper.calibrate()
                rospy.sleep(sleep_time)
            else:
                rospy.loginfo("%s already calibrated." %name)

        except Exception, e:
            rospy.logerr(e.strerror)
            rospy.logerr("Calibration failed")
            return False

        return True

    def enable_arms(self):
        """
        Enables the robot which enables the arms

        :return: bool
        """

        rospy.loginfo("Attempting to enabling robot.")
        rs = RobotEnable(CHECK_VERSION)

        try:
            rs.enable()
        except Exception, e:
            rospy.logerr(e.strerror)
            rospy.logerr("Failed to enable arms.")
            return False

        rospy.loginfo("Successfully enabled robot.")
        return True

    def untuck_arms(self):
        """
        Untucks the arms, if tucked otherwise returns them to neutral untucked position.

        :return:
        """

        rospy.loginfo("Returning arms back to untucked position.")
        tucker = tuck_arms.Tuck(False)
        rospy.on_shutdown(tucker.clean_shutdown)
        try:
            tucker.supervised_tuck()
        except Exception, e:
            rospy.logerr(e.strerror)
            rospy.logerr("Failed to return arms to untucked position.")
            return False

        rospy.loginfo("Arms in untucked position.")

        rospy.loginfo("Moving arms to neutral position.")
        left_arm_neutral = rospy.get_param(LEFT_ARM_NEUTRAL)
        right_arm_neutral = rospy.get_param(RIGHT_ARM_NEUTRAL)
        arm_l = Limb("left")
        arm_r = Limb("right")

        arm_l.move_to_joint_positions(left_arm_neutral)
        arm_r.move_to_joint_positions(right_arm_neutral)

        return True

    def callback(self, request=True):
        """
        Checks the status of the grasping components.

        :param request: request from brain to check the handling
        :return: True if all checks are good and 'False' otherwise
        """

        # 1. Enable Arms
        if not self.enable_arms():
            return False

        # 2. Untuck Arms
        if not self.untuck_arms():
            return False

        # 3. Gripper Calibration
        gripper = Gripper('left')
        if not self.calibrate_grippers(gripper):
            return False
        gripper.open(block=True)

        gripper = Gripper('right')
        if not self.calibrate_grippers(gripper):
            return False
        gripper.open(block=True)

        rospy.loginfo("All grasping checks were completed successfully.")
        return True


if __name__ == "__main__":
    rospy.init_node("grasping_status")
    server = GraspingStatusServer("grasping_status_service", GraspingStatus)

    rospy.logdebug("The grasping status Service is running.")
    rospy.spin()
