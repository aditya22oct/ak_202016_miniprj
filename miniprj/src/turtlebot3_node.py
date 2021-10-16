#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Turtlebot3_Node(object):

    def __init__(self):
        '''
        Class constructor
        '''
        self.node = rospy.init_node('Turtlebot3_Node')
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.cb_sub_scan)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.vel_val = Twist()
        self.regions = {}
        self.rate = rospy.Rate(4)
        self.min_dist_to_obs = 1.5 # unit m
        self.safe_dist = 2

    def cb_sub_scan(self, msg_scan):
        '''
        Subscriber Callback method for /scan topic
        '''
        laser_dist_data = list(msg_scan.ranges)

        # 180 degree region in front of robot is divided into 5 parts
        # to know the position of obstacle
        front_region = laser_dist_data[0:18] + laser_dist_data[342:]
        left_front_region = laser_dist_data[18:54]
        left_region = laser_dist_data[54:90]
        right_front_region = laser_dist_data[306:342]
        right_region = laser_dist_data[270:306]

        self.regions = {'front_region': min([min(front_region),10]),
        'left_front_region': min([min(left_front_region),10]),
        'left_region': min([min(left_region),10]),
        'right_front_region': min([min(right_front_region),10]),
        'right_region': min([min(right_region),10]),
        }

    def move_fwd(self):
        '''
        This methods moves robot in straight fwd direction
        '''
        self.vel_val.linear.x = 0.1
        self.vel_val.angular.z = 0.0
        self.pub_cmd_vel.publish(self.vel_val)
        rospy.loginfo('Moving Forward')

    def stop_robot(self):
        '''
        This methods stops robot
        '''
        self.vel_val.linear.x = 0.0
        self.vel_val.angular.z = 0.0
        self.pub_cmd_vel.publish(self.vel_val)
        rospy.loginfo('Stopping')

    def turn_left(self):
        '''
        This methods turns robot to left
        '''
        self.vel_val.linear.x = 0.0
        self.vel_val.angular.z = 0.1
        self.pub_cmd_vel.publish(self.vel_val)
        rospy.loginfo('Turning Left')

    def turn_right(self):
        '''
        This methods turns robot to right
        '''
        self.vel_val.linear.x = 0.0
        self.vel_val.angular.z = -0.1
        self.pub_cmd_vel.publish(self.vel_val)
        rospy.loginfo('Turning Right')

    def start_find_gap(self):
        '''
        This methods starts the find gap task to be performed by turtlebot
        '''
        # Delay added to get the laser sensor data
        self.rate.sleep()

        while not rospy.is_shutdown():
            if self.regions['front_region'] > self.min_dist_to_obs:
                self.move_fwd()
            elif self.regions['front_region'] < self.min_dist_to_obs:
                if self.regions['left_front_region'] > self.safe_dist and self.regions['right_front_region'] < self.safe_dist:
                    self.turn_left()
                elif self.regions['left_front_region'] < self.safe_dist and self.regions['right_front_region'] > self.safe_dist:
                    self.turn_right()
                elif self.regions['left_front_region'] > self.safe_dist and self.regions['right_front_region'] > self.safe_dist:
                    self.turn_left()
                elif self.regions['left_front_region'] < self.safe_dist and self.regions['right_front_region'] < self.safe_dist:
                    self.stop_robot()
            self.rate.sleep()


if __name__ == '__main__':
    robot = Turtlebot3_Node()
    robot.start_find_gap()
