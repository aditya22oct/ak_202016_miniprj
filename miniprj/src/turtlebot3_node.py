#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
import math

class Turtlebot3_Node(object):

    def __init__(self):
        '''
        Class constructor
        '''
        self.node = rospy.init_node('Turtlebot3_Node')
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.cb_sub_scan)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cb_odom)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.vel_val = Twist()
        self.regions = {}
        self.rate = rospy.Rate(10)
        self.min_dist_to_obs = 1.5 # unit m
        self.safe_dist = 5
        self.curr_pose = None
        self.curr_orient = None
        self.orient_acc = 0.8
        self.forward_start = True
        self.fwd_orient = None
        self.state = 1

    def cb_sub_scan(self, msg_scan):
        '''
        Subscriber Callback method for /scan topic
        '''
        laser_dist_data = list(msg_scan.ranges)

        # 180 degree region in front of robot is divided into 5 parts
        # to know the position of obstacle
        front_region = laser_dist_data[0:18] + laser_dist_data[342:]
        left_front_region = laser_dist_data[18:54]
        left_region = laser_dist_data[54:108]
        right_front_region = laser_dist_data[306:342]
        right_region = laser_dist_data[262:306]

        self.regions = {'front_region': min([min(front_region),10]),
        'left_front_region': min([min(left_front_region),10]),
        'left_region': min([min(left_region),10]),
        'right_front_region': min([min(right_front_region),10]),
        'right_region': min([min(right_region),10]),
        }

    def cb_odom(self, msg_odom):
        '''
        Subscriber Callback mathod for /odom topic
        '''

        self.current_pose = msg_odom.pose.pose.position

        curr_quat_x = msg_odom.pose.pose.orientation.x
        curr_quat_y = msg_odom.pose.pose.orientation.y
        curr_quat_z = msg_odom.pose.pose.orientation.z
        curr_quat_w = msg_odom.pose.pose.orientation.w

        curr_quat = [curr_quat_x,
                     curr_quat_y,
                     curr_quat_z,
                     curr_quat_w
                     ]

        # converting quaternions into euler angles
        curr_euler = euler_from_quaternion(curr_quat)

        # storing the jaw angle i.e. orientation of robot along Z-axis
        self.curr_orient = curr_euler[2]

        # Converting angle in radians to degrees
        self.curr_orient = self.curr_orient * (180 / math.pi)


    def move_fwd(self):
        '''
        This methods moves robot in straight fwd direction
        '''

        if self.forward_start:
            print("Entered if loop")
            self.fwd_orient = self.curr_orient
            print("Orientation: {}".format(self.fwd_orient))
            self.forward_start = False

        orien_diff = self.fwd_orient  - self.curr_orient

        if abs(orien_diff) > self.orient_acc:
            if orien_diff > 0:
                self.vel_val.angular.z = 0.005
            else:
                self.vel_val.angular.z = -0.005

        self.vel_val.linear.x = 0.1

        rospy.loginfo('Moving Forward')

        if self.regions['front_region'] < self.min_dist_to_obs:
            print("wall found")
            self.state = 2

        print("Desired Orientation: {}".format(self.fwd_orient))
        print("Current Orientation: {}".format(self.curr_orient))

    def stop_robot(self):
        '''
        This methods stops robot
        '''
        self.vel_val.linear.x = 0.0
        self.vel_val.angular.z = 0.0

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
        This method starts the find gap task to be performed by turtlebot
        '''
        # Delay added to get the laser sensor data
        time.sleep(0.25)

        while not rospy.is_shutdown():
            # if self.regions['front_region'] > self.min_dist_to_obs:
            #     self.move_fwd()
            # elif self.regions['front_region'] < self.min_dist_to_obs:
            #     if self.regions['left_front_region'] > self.safe_dist and self.regions['right_front_region'] < self.safe_dist:
            #         self.turn_left()
            #     elif self.regions['left_front_region'] < self.safe_dist and self.regions['right_front_region'] > self.safe_dist:
            #         self.turn_right()
            #     elif self.regions['left_front_region'] > self.safe_dist and self.regions['right_front_region'] > self.safe_dist:
            #         self.turn_left()
            #     elif self.regions['left_front_region'] < self.safe_dist and self.regions['right_front_region'] < self.safe_dist:
            #         self.stop_robot()
            self.move_fwd()
            self.rate.sleep()

    def adjust_orientation(self, desired_orient):
        '''
        This method adjusts the orientation of turtlebot to give value

        desired_orient[float]: desired orientation
        '''

        # Delay added to get the odom data
        time.sleep(0.25)

        while not rospy.is_shutdown():

            diff_orient = desired_orient - self.curr_orient
            print(diff_orient)
            if abs(diff_orient) > self.orient_acc:
                if diff_orient > 0:
                    self.turn_left()
                elif diff_orient < 0:
                    self.turn_right()
                else:
                    self.stop_robot()

            else:
                self.stop_robot()

            self.rate.sleep()

    def follow_wall(self):
        '''
        This method follows the wall without colliding it.
        '''
        time.sleep(0.25)

        if self.regions['right_region'] < self.safe_dist:
            self.move_fwd()

        else:
            self.stop_robot()

        print(self.forward_start)
        self.forward_start = True
        print(self.forward_start)

    def start_task(self):
        '''
        This method starts the desired ask
        '''

        time.sleep(0.25)

        print(self.state)

        while True:

            print('in while loop')
            print(self.state)

            if self.state == 1:
                self.move_fwd()

            elif self.state == 2:
                self.stop_robot()


            self.pub_cmd_vel.publish(self.vel_val)
            self.rate.sleep()



if __name__ == '__main__':
    robot = Turtlebot3_Node()
    # robot.start_find_gap()
    # robot.adjust_orientation(110)
    robot.start_task()
