#!/usr/bin/env python
"""
This code converts pose to odometry data.
It takes geometry_msgs/PoseWithCovarianceStamped data from topic /poseupdate and publishes nav_msgs/Odometry data to topic /odom.
By comparing consecutive two poses and differentiate their x, y and yaw, then do simple trinometry to find right linear vel_x and direction of velocity.

Author: Nico Smuts
Date: 05/18
"""

import math

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3, PoseWithCovarianceStamped

odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
odom_broadcaster = tf.TransformBroadcaster()

vel_x = 0.0
vel_yaw = 0.0


def velocity_cb(vel_data):
    """
    Grabs the forward speed and angular speed about the vertical axis from velocity data published on 'cmd_vel' topic.
    :param vel_data: geometry_msgs/Twist, velocity commands from hector_mapping package.
    :return:
    """
    global vel_x
    global vel_yaw
    vel_x = vel_data.linear.x
    vel_yaw = vel_data.angular.z


def pose_update_cb(pose_data):
    """
    Publishes odometry information
    :param pose_data: geometry_msgs/PoseWithCovarianceStamped, current pose data from hector_mapping package.
    :return:
    """
    # Broadcast transform base_link -> odom
    odom_broadcaster.sendTransform(
        (pose_data.pose.pose.position.x, pose_data.pose.pose.position.y, pose_data.pose.pose.position.z),
        (pose_data.pose.pose.orientation.x, pose_data.pose.pose.orientation.y, pose_data.pose.pose.orientation.z,
         pose_data.pose.pose.orientation.w),
        pose_data.header.stamp,
        "base_link",
        "odom"
    )

    # Next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header= pose_data.header
    odom.header.frame_id = "odom"

    # Set the position
    odom.pose = pose_data.pose

    # Set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vel_x, 0, 0), Vector3(0, 0, vel_yaw))

    # Publish the message
    odom_pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node('odometry_publisher_cmd_vel')
    sub_pose = rospy.Subscriber('poseupdate', PoseWithCovarianceStamped, pose_update_cb)
    sub__cmd_vel = rospy.Subscriber('cmd_vel', Twist, velocity_cb)
    rospy.spin()
