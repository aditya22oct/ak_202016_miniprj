#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math


def cb_odom(msg):
    '''
    Subscriber Callback function for /odom topic

    msg(Odometry): Message subscribed from the /odom topic
    '''
    current_pose = msg.pose.pose.position
    print(current_pose)

    curr_quat_x = msg.pose.pose.orientation.x
    curr_quat_y = msg.pose.pose.orientation.y
    curr_quat_z = msg.pose.pose.orientation.z
    curr_quat_w = msg.pose.pose.orientation.w

    curr_quat = [curr_quat_x,
                 curr_quat_y,
                 curr_quat_z,
                 curr_quat_w
                 ]

    # converting quaternions into euler angles
    curr_euler = euler_from_quaternion(curr_quat)

    # storing the jaw angle i.e. orientation of robot along Z-axis
    current_orientation = curr_euler[2]

    # Converting angle in radians to degrees
    current_orientation = current_orientation * (180 / math.pi)
    print(current_orientation)


def main():
    rospy.init_node('odom_topic_subscriber')
    sub_odom = rospy.Subscriber('/odom', Odometry, cb_odom)
    rospy.spin()


if __name__ == '__main__':
    main()
