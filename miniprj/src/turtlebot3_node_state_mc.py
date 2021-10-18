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
        self.curr_pose = None
        self.curr_orient = None
        self.orient_acc = 0.8
        self.orient_achieved = False
        self.min_dist_to_obs = 1
        self.previous_state = 1
        self.state = 1
        self.next_state = 1

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

    def change_state(self, state):
        '''
        This method changes the state of the robot.

        state(int): Next state
        '''
        if self.state != state:
            self.previous_state = self.state
            self.state = state
            self.orient_achieved = False

    def stop_robot(self):
        '''
        This methods stops robot
        '''
        self.vel_val.linear.x = 0.0
        self.vel_val.angular.z = 0.0

        # self.change_state(3)

    def turn_left(self):
        '''
        This methods turns robot to left
        '''
        self.vel_val.linear.x = 0.0
        self.vel_val.angular.z = 0.175

    def turn_right(self):
        '''
        This methods turns robot to right
        '''
        self.vel_val.linear.x = 0.0
        self.vel_val.angular.z = -0.175

    def move_fwd(self):
        '''
        This methods moves robot in straight fwd direction

        State : 1
        '''

        self.vel_val.linear.x = 0.2
        rospy.loginfo('Moving Forward')

        # if self.regions['front_region'] < self.min_dist_to_obs:
        #     rospy.loginfo("obstacle Found!!!")
        #     self.stop_robot()
        #     self.change_state(2)

    def adjust_orientation(self, desired_orient):
        '''
        This method adjusts the orientation of turtlebot to give value

        desired_orient[float]: desired orientation
        '''

        # Delay added to get the odom data
        time.sleep(0.25)

        diff_orient = desired_orient - self.curr_orient
        print(diff_orient)
        if abs(diff_orient) > self.orient_acc:
            rospy.loginfo("Adjusting Orientation")
            if diff_orient > 0:
                self.turn_left()
            elif diff_orient < 0:
                self.turn_right()
            else:
                self.stop_robot()
        else:
            self.orient_achieved = True

    def follow_wall(self):
        '''
        This method will help robot follow the wall till gap is found
        '''
        self.vel_val.linear.x = 0.2
        rospy.loginfo('Following wall')

        if self.regions['right_region'] > 2.5:
            self.stop_robot()
            self.change_state(4)
        else:
            self.move_fwd()

    def state_transition(self):

        if (self.regions['front_region'] > self.min_dist_to_obs) and (self.previous_state == 1) and (self.state == 1):
            self.change_state(1)
        elif (self.regions['front_region'] < self.min_dist_to_obs) and (self.state == 1):
            self.change_state(2)
        elif (self.previous_state == 1) and (self.state == 2):
            self.change_state(3)
        elif self.orient_achieved and (self.previous_state == 2):
            self.change_state(2)
        elif (self.previous_state == 3) and (self.state == 2):
            self.change_state(4)
        elif (self.regions['right_region'] > 3) and (self.previous_state == 2):
            self.change_state(2)
        elif (self.previous_state == 4) and (self.state == 2):
            self.change_state(5)
        # elif (self.previous_state == 5) and (self.state == 2):
        #     self.change_state(1)


    def start_task(self):
        '''
        This method starts the desired ask
        '''

        time.sleep(0.25)

        while True:

            print('in while loop')
            print(self.state)
            self.state_transition()

            if self.state == 1:
                self.move_fwd()

            elif self.state == 2:
                self.stop_robot()

            elif self.state == 3:
                self.adjust_orientation(90)

            elif self.state == 4:
                self.follow_wall()

            self.pub_cmd_vel.publish(self.vel_val)
            self.rate.sleep()



if __name__ == '__main__':
    robot = Turtlebot3_Node()
    robot.start_task()
