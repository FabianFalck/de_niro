#!/usr/bin/env python
"""
This code takes geometry_msgs/Twist message from /cmd_vel topic and publishes Float32MultiArray message to /base_control_sig topic.
Linear remapping from cmd_vel velocity to base_control_sig velocity with all parameters pre-set.
New feature LOW_THRESHOLD is introduced so that any linear velocity smaller than low_threshold will be truncated to 0

Author: -
Date: -
"""

import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

# Parameters need to be tuned
LINEAR_MAX_FORWARD_SPEED = 60.0
LINEAR_MIN_FORWARD_SPEED = 20.0
LINEAR_MAX_BACKWARD_SPEED = 30.0
LINEAR_MIN_BACKWARD_SPEED = 20.0
ANGULAR_MAX_SPEED = 100.0
ANGULAR_MIN_SPEED = 20.0

LINEAR_MAX_SPEED_CMD_VEL = 0.6 # Should match max_vel_x value in teb_local_planner_params.yaml
LINEAR_MIN_SPEED_CMD_VEL = 0.0
ANGULAR_MAX_SPEED_CMD_VEL = 0.6 # Should match max_vel_theta value in teb_local_planner_params.yaml
ANGULAR_MIN_SPEED_CMD_VEL = 0.0

# Constant ratios
LINEAR_FORWARD_SPEED_RANGE = LINEAR_MAX_FORWARD_SPEED #- LINEAR_MIN_FORWARD_SPEED
LINEAR_BACKWARD_SPEED_RANGE = LINEAR_MAX_BACKWARD_SPEED #- LINEAR_MIN_BACKWARD_SPEED
LINEAR_SPEED_CMD_VEL_RANGE = LINEAR_MAX_SPEED_CMD_VEL - LINEAR_MIN_SPEED_CMD_VEL

ANGULAR_SPEED_RANGE = ANGULAR_MAX_SPEED #- ANGULAR_MIN_SPEED
ANGULAR_SPEED_CMD_VEL_RANGE = ANGULAR_MAX_SPEED_CMD_VEL #- ANGULAR_MIN_SPEED_CMD_VEL

LINEAR_SPEED_RATIO = LINEAR_FORWARD_SPEED_RANGE / LINEAR_SPEED_CMD_VEL_RANGE
ANGULAR_SPEED_RATIO = ANGULAR_SPEED_RANGE / ANGULAR_SPEED_CMD_VEL_RANGE


def converter_cb(message):
    """
    Converts velocity from 'cmd_vel' topic into a format that is understood by the Mbed on the base.
    :param message: geometry_msgs/Twist, from 'cmd_vel' topic
    :return: Float32MultiArray
    """
    vel_data = Float32MultiArray()
    vel_data.data = [0.0,0.0]
    rospy.loginfo("cmd_vel values: %s, %s", message.linear, message.angular)
    linear_speed_cmd_vel = message.linear.x
    angular_speed_cmd_vel = -message.angular.z

    # Linear speed
    if math.fabs(linear_speed_cmd_vel) <= 0:
        linear_speed = 0

    elif linear_speed_cmd_vel > 0:
        linear_speed = linear_speed_cmd_vel * LINEAR_SPEED_RATIO
        if linear_speed < LINEAR_MIN_FORWARD_SPEED:
            linear_speed = LINEAR_MIN_FORWARD_SPEED

    else:
        linear_speed = linear_speed_cmd_vel * LINEAR_SPEED_RATIO #- LINEAR_MIN_BACKWARD_SPEED
        if linear_speed > -LINEAR_MIN_BACKWARD_SPEED:
            linear_speed = -LINEAR_MIN_BACKWARD_SPEED
        angular_speed_cmd_vel = -angular_speed_cmd_vel #negate angle when reversing

    # Angular speed
    if math.fabs(angular_speed_cmd_vel) == 0:
        angular_speed = 0

    elif angular_speed_cmd_vel < 0:
        angular_speed = angular_speed_cmd_vel * ANGULAR_SPEED_RATIO
        if angular_speed > -ANGULAR_MIN_SPEED:# and (linear_speed >= -LINEAR_MIN_BACKWARD_SPEED) and (linear_speed <= LINEAR_MIN_FORWARD_SPEED):
            angular_speed = -ANGULAR_MIN_SPEED

    else:
        angular_speed = angular_speed_cmd_vel * ANGULAR_SPEED_RATIO
        if angular_speed < ANGULAR_MIN_SPEED:# and (linear_speed >= -LINEAR_MIN_BACKWARD_SPEED) and (linear_speed <= LINEAR_MIN_FORWARD_SPEED): #Enforce minimum angular speed only if linear speed is also small
            angular_speed = ANGULAR_MIN_SPEED

    # Cap speed
    if linear_speed > LINEAR_MAX_FORWARD_SPEED:
        linear_speed = LINEAR_MAX_FORWARD_SPEED
    elif linear_speed < -LINEAR_MAX_BACKWARD_SPEED:
        linear_speed = -LINEAR_MAX_BACKWARD_SPEED

    if angular_speed > ANGULAR_MAX_SPEED:
        angular_speed = ANGULAR_MAX_SPEED
    elif angular_speed < -ANGULAR_MAX_SPEED:
        angular_speed = -ANGULAR_MAX_SPEED

    vel_data.data = [linear_speed,angular_speed]
    rospy.loginfo('lin vel: %s  ang vel: %s', linear_speed, angular_speed)

    pub_base_control_sig.publish(vel_data)


if __name__ == "__main__":
    rospy.init_node('vel_topic_converter')
    pub_base_control_sig = rospy.Publisher('base_control_sig', Float32MultiArray, queue_size=1)
    sub = rospy.Subscriber('cmd_vel', Twist, converter_cb)
    rospy.spin()
