#!/usr/bin/env python
"""
Server code for the Grasping Handler service.
This service is responsible for operating DE NIRO's arms when requested to pick up an object.

Request: geometry_msgs/Point
Response: unit8

Author: John Lingi & Kim Rants
Date: 05/2018
"""

import rospy
from grasping.srv import GraspingHandler, GraspingHandlerResponse
from geometry_msgs.msg import Point
from ik_service_client import get_joint_angles
import baxter_interface
import tuck_arms

import sys, os
root_path = "/home/robin/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
from ros_server import ROSServiceServer


LIMB_TO_MOVE = "limb_to_move"
SLEEP_TIME = 0.3
LEFT_ARM_NEUTRAL = "left_arm_neutral"
RIGHT_ARM_NEUTRAL = "right_arm_neutral"

def openclose_gripper(gripper, cmd):
    """
    Function to open or close the gripper. Sleep is necessary.

    :param gripper: baxter_interface.Gripper object
    :param cmd: String "open" or "close"
    :return:
    """
    if cmd == "open":
        gripper.open(block=True)
    elif cmd == "close":
        gripper.close(block=True)
    else:
        rospy.logerr("Bad command to openclose_gripper, options are 'open' nor 'close'")

    rospy.sleep(SLEEP_TIME)


def return_to_untucked(arm, limb):
    """
    Returns the arms back to the untucked position.

    :return: None
    """

    # rospy.loginfo("Returning arms back to untucked position")
    # tucker = tuck_arms.Tuck(False)
    # rospy.on_shutdown(tucker.clean_shutdown)
    # tucker.supervised_tuck()
    # rospy.sleep(SLEEP_TIME)
    if limb == "right":
        joints = rospy.get_param(RIGHT_ARM_NEUTRAL)
    else:
        joints = rospy.get_param(LEFT_ARM_NEUTRAL)

    arm.move_to_joint_positions(joints)



class GraspingHandlerServer(ROSServiceServer):

    def transform_request_coordinates(self, raw_coordinate):
        """
        Transforms coordinate from kinect coordinate frame to robot coordinate frame

        For DE NIRO: x is forward, y is left, z is up
        For Kinect: x is right, y is down, z is forward

        :param raw_coordinate: geometry_msg/Point object (x, y, z) in kinect frame (raw)
        :return: geometry_msg/Point object (x, y, z) in robot frame (transformed)
        """
        offset_kinect_to_DN_x = 0.30381  # ADD THIS TO DENIRO X
        offset_kinect_to_DN_y = -0.092  # ADD THIS TO DENIRO Y
        offset_kinect_to_DN_z = 0.83  # ADD THIS TO DENIRO Z

        # Inverting dimensions and mapping to the ROS base coordinate system
        read_x = raw_coordinate.z  # DE NIRO base x = kinect z (plus some offset)
        read_y = -raw_coordinate.x  # DE NIRO base y = inverted Kinect x (plus some offset)
        read_z = -raw_coordinate.y  # DE NIRO base z = inverted Kinect y (plus some offset)

        # Putting all offsets together with the calibration to get final destination
        arm_destination = Point(0, 0, 0)
        arm_destination.x = read_x + offset_kinect_to_DN_x
        arm_destination.y = read_y + offset_kinect_to_DN_y
        arm_destination.z = read_z + offset_kinect_to_DN_z

        return arm_destination

    def get_intermediate_point(self, final_destination, offset=[-0.15, 0, 0]):
        """
        Adds offset to the final destination

        :param final_destination: geometry_msgs/Point object
        :param offset: how much to add to final position. Double array [x, y, z] offsets
        :return: geometry_msgs/Point
        """
        intermediate_point = Point(0, 0, 0)

        intermediate_point.x = final_destination.x + offset[0]
        intermediate_point.y = final_destination.y + offset[1]
        intermediate_point.z = final_destination.z + offset[2]

        return intermediate_point

    def move_to_hero(self, baxter_arm, limb):
        """
        Moves specified limb to the hero position

        :param baxter_arm: arm to move
        :param limb: which arm, 'right' or 'left
        :return:
        """

        rospy.loginfo("Moving %s to hero position." % limb)
        if limb == "left":
            hero_position_joints = {"left_w0": 0.06941263065181497, "left_w1": 0.15263108839459866,
                                      "left_w2": -0.23009711818281206, "left_e0": -1.3940050409908697,
                                      "left_e1": -0.020708740636453084, "left_s0": 0.14189322287940076,
                                      "left_s1": -1.2333205534598726}

            baxter_arm.move_to_joint_positions(hero_position_joints)

        elif limb == "right":
            hero_position_joints = {"right_s0": -0.6841554313968945, "right_s1": -1.4196992191879505,
                                       "right_w0": 0.6807039746241523, "right_w1": 0.15378157398551273,
                                       "right_w2": -0.7244224270788866, "right_e0": 1.6045439041281429,
                                       "right_e1": 0.3451456772742181}

            baxter_arm.move_to_joint_positions(hero_position_joints)
        else:
            rospy.logerr("Bad input to function move_to_hero. %s is not a valid limb parameter" % limb)

    def callback(self, request):
        """
        Moves the arm to the specified object location via an intermediate point. This function uses inverse kinematics
        to solve for the joint angles of the final pose. If a solution exists, DE NIRO will move the specified arm to the
        position received, grip the object and move his arm into HERO pose (point up to the sky).

        WARNING: Before this function is called, make sure to calibrate
        the grippers.

        :param request: geometry_msgs/Point: contains x, y, z coordinates of object in Kinect's frame of reference
        :return: uint8. 0 = could not reach the desired location, 1 = successful grab, 2 = could reach but object not at location
        """

        end_point = self.transform_request_coordinates(request.object_position)
        limb = rospy.get_param(LIMB_TO_MOVE)
        arm = baxter_interface.Limb(limb)
        gripper = baxter_interface.Gripper(limb)

        # Success / Fail conditions
        CANT_REACH = 0
        OBJECT_GRABBED = 1
        OBJECT_MOVED = 2

        offset = [-0.15, 0, 0] # [x, y, z]
        arm_intermediate = self.get_intermediate_point(end_point, offset)

        intermediate_limb_joints = get_joint_angles(arm_intermediate.x,
                                                    arm_intermediate.y,
                                                    arm_intermediate.z)

        rospy.loginfo("INTERMEDIATE STEP")
        rospy.loginfo("intermediate_limb_joints = %s" % intermediate_limb_joints)

        if intermediate_limb_joints == {}:
            rospy.loginfo("No intermediate joint angles found.")
            return CANT_REACH

        rospy.loginfo("Moving arm to intermediate position.")
        arm.move_to_joint_positions(intermediate_limb_joints)
        rospy.sleep(SLEEP_TIME)

        rospy.loginfo("Searching for final position joint angles.")
        final_limb_joints = get_joint_angles(end_point.x,
                                            end_point.y,
                                            end_point.z)

        rospy.loginfo("limb_joints = %s" % final_limb_joints)

        if final_limb_joints == {}:
            rospy.loginfo("No joint angles found.")
            return_to_untucked(arm, limb)
            return CANT_REACH

        rospy.loginfo("Moving arm into position to grab object.")
        arm.move_to_joint_positions(final_limb_joints)
        rospy.sleep(SLEEP_TIME)

        # Check gripper open first
        openclose_gripper(gripper, cmd='open')
        rospy.sleep(SLEEP_TIME)

        rospy.loginfo("Closing my gripper")
        openclose_gripper(gripper, cmd='close')
        rospy.sleep(SLEEP_TIME)

        rospy.loginfo("Moving arm back into intermediate")
        rospy.loginfo("intermediate_limb_joints = %s" % intermediate_limb_joints)
        arm.move_to_joint_positions(intermediate_limb_joints)
        rospy.sleep(SLEEP_TIME)

        # Dynamic Grasping
        grip_failure_pos = 10
        if gripper.position() < grip_failure_pos:
            return_to_untucked(arm, limb)
            openclose_gripper(gripper, cmd='open')
            rospy.sleep(SLEEP_TIME)
            return OBJECT_MOVED

        self.move_to_hero(arm, limb)
        rospy.sleep(SLEEP_TIME)

        return OBJECT_GRABBED


if __name__ == "__main__":
    rospy.init_node("grasping_handler")
    grasping_handler_server = GraspingHandlerServer("grasping_handler_service", GraspingHandler)

    rospy.loginfo("The grasping handler server is running.")
    rospy.spin()
