#!/usr/bin/python3

import rospy
import math
import message_filters
import random

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf2_ros
from diff_drive import diff_drive

from tf.transformations import euler_from_quaternion

class Control():
    def __init__(self):
        rospy.init_node('control_2')
        self.robot1_odom = None
        self.robot5_odom = None
        
        robot1_odom_sub = message_filters.Subscriber('/robot1/odometry/filtered', Odometry)
        robot5_odom_sub = message_filters.Subscriber('/robot5/odometry/filtered', Odometry)
        subscribers = message_filters.ApproximateTimeSynchronizer([robot1_odom_sub, robot5_odom_sub], queue_size=10, slop=0.9,allow_headerless=False)
        subscribers.registerCallback(self.callback)

        self.vel_pub_fl = rospy.Publisher('/robot5/wheel_fl_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub_fr = rospy.Publisher('/robot5/wheel_fr_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub_bl = rospy.Publisher('/robot5/wheel_bl_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub_br = rospy.Publisher('/robot5/wheel_br_velocity_controller/command', Float64, queue_size=10)
       
        self.vel_pub_l = rospy.Publisher('/robot1/wheel_l_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub_r = rospy.Publisher('/robot1/wheel_r_velocity_controller/command', Float64, queue_size=10)


        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            rospy.spin()
    # Getting the robots values 
    def callback(self, robot1_msg, robot5_msg):
        self.robot1_odom = robot1_msg
        self.robot5_odom = robot5_msg
        self.pit_pushing()

    def control_logic(self):
        if self.robot1_odom != None and self.robot5_odom != None:
            
            robot5_pos =  self.robot5_odom.pose.pose.position
            robot5_ori =  self.robot5_odom.pose.pose.orientation
            robot5_q = [robot5_ori.x, robot5_ori.y, robot5_ori.z, robot5_ori.w]
            robot5_roll, robot5_pitch, robot5_yaw = euler_from_quaternion(robot5_q)

            robot1_pos =  self.robot1_odom.pose.pose.position
            robot1_ori =  self.robot1_odom.pose.pose.orientation
            robot1_q = [robot1_ori.x, robot1_ori.y, robot1_ori.z, robot1_ori.w]
            robot1_roll, robot1_pitch, robot1_yaw = euler_from_quaternion(robot5_q)
            
            direction = { # If robot5 is going in direction returns True
                'east' :  math.pi/2 > robot5_yaw and -math.pi/2 < robot5_yaw,
                'west' :  math.pi/2 < robot5_yaw or -math.pi/2 > robot5_yaw,
                'north' : math.pi > robot5_yaw > 0,
                'south' : -math.pi < robot5_yaw < 0
            }

            if (
            robot5_pos.x > 2.7 and direction['east'] or
            robot5_pos.x < 0.5 and direction['west'] or
            robot5_pos.y > 2.7 and direction['north'] or
            robot5_pos.y < 0.5 and direction['south']):
                diff_drive(self, 0.0, 1.0)
            else:
                diff_drive(self, 1.0, 0.0)


    def pit_pushing(self):
        robot5_pos =  self.robot5_odom.pose.pose.position
        robot5_ori =  self.robot5_odom.pose.pose.orientation
        robot5_q = [robot5_ori.x, robot5_ori.y, robot5_ori.z, robot5_ori.w]
        robot5_roll, robot5_pitch, robot5_yaw = euler_from_quaternion(robot5_q)

        robot1_pos =  self.robot1_odom.pose.pose.position
        robot1_ori =  self.robot1_odom.pose.pose.orientation
        robot1_q = [robot1_ori.x, robot1_ori.y, robot1_ori.z, robot1_ori.w]
        robot1_roll, robot1_pitch, robot1_yaw = euler_from_quaternion(robot5_q)
        
        pit_pos_x = 0.57
        pit_pos_y = 0.65
        
        distance = 0.2
        slope = (robot1_pos.y - pit_pos_y) / (robot1_pos.x - pit_pos_x)
        end_point_x = robot1_pos.x + distance
        end_point_y = robot1_pos.y + slope
        print(f"Robot1 position: {robot1_pos}")
        print(f"Robot5 position: {robot5_pos}")

        angle_to_pit = math.atan2((end_point_y - robot5_pos.y), (end_point_x - robot5_pos.x))

        if abs(angle_to_pit - robot5_yaw) > 0.5:
            diff_drive(self, 0, 4)
        else:
            diff_drive(self, 4, 0)

    def shutdown(self):
        diff_drive(self, 0.0, 0.0)

def main():
    Control()

if __name__ == "__main__":
    main()
