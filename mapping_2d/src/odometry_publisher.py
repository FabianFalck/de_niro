#!/usr/bin/env python
"""
This code converts pose to odometry data.
It takes geometry_msgs/PoseWithCovarianceStamped data from topic /poseupdate and publishes nav_msgs/Odometry data to topic /odom
By comparing consetutive two poses and differentiate their x, y and yaw, then do simple trinometry to find right linear vel_x and direction of velocity.

Author: Nico Smuts
Date: 05/18
"""
import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

# Global last_x
last_x = 0.0

# Global last_y
last_y = 0.0

# Global last_yaw
last_yaw = 0.0

# Global last_t
last_t = 0.0


def odom_pub_cb(message):
    """
    A callback function that subscribes to the 'poseupdate' topic and publishes odometry information on the 'odom' topic.
    The 'poseupdate' topic is published by the hector_mapping package.

    :param message: PoseWithCovarianceStamped object
    :return:
    """
    global last_x
    global last_y
    global last_t
    global last_yaw

    msg_quaternion = message.pose.pose.orientation

    quaternion = (
        msg_quaternion.x,
        msg_quaternion.y,
        msg_quaternion.z,
        msg_quaternion.w
    )

    # Get euler from quaternion
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # Do differentiation
    current_yaw = euler[2]
    current_x = message.pose.pose.position.x
    current_y = message.pose.pose.position.y
    current_t = message.header.stamp.to_sec()

    diff_t = current_t - last_t
    if diff_t == 0:
        diff_t = 1

    vel_x = (current_x - last_x) / diff_t
    vel_y = (current_y - last_y) / diff_t
    theta = math.atan2(vel_y, vel_x)

    rospy.loginfo(theta)
    rospy.loginfo(current_yaw)

    # Get velocity
    vel_x = math.sqrt(math.pow(vel_x, 2) + math.pow(vel_y, 2))
    vel_yaw = (current_yaw - last_yaw) / diff_t
    if math.fabs(current_yaw - theta) > 1.57079:
        vel_x = -vel_x

    # Update last values
    last_x = current_x
    last_y = current_y
    last_t = current_t
    last_yaw = current_yaw

    # Broadcast transform base_link -> odom
    odom_broadcaster.sendTransform(
        (message.pose.pose.position.x,message.pose.pose.position.y,message.pose.pose.position.z),
        (message.pose.pose.orientation.x,message.pose.pose.orientation.y,message.pose.pose.orientation.z,message.pose.pose.orientation.w),
        message.header.stamp,
        "base_link",
        "odom"
    )

    # Next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header = message.header
    odom.header.frame_id = "odom"

    # Set the position
    odom.pose = message.pose

    # Set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vel_x, 0, 0), Vector3(0, 0, vel_yaw))

    # Publish the odometry information
    odom_pub.publish(odom)


if __name__ == "__name__":
    rospy.init_node("odometry_publisher")
    sub = rospy.Subscriber("poseupdate", PoseWithCovarianceStamped, odom_pub_cb) # Amcl_pose from amcl
    rospy.spin()