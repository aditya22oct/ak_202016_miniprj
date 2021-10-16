#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


def main():
    '''
    function the published required velocity into the
    /cmd_vel topic

    '''

    rospy.init_node('cmd_vel_publisher')
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    r = rospy.Rate(10) # 10 Hz
    vel_value = Twist() # Creating Object of Twist Type

    vel_value.linear.x = 0.5 # value is in m/s
    vel_value.angular.z = 0.5 # value is in rad/s
    
    while not rospy.is_shutdown():

        pub_cmd_vel.publish(vel_value)
        r.sleep()


if __name__ == '__main__':
    main()
