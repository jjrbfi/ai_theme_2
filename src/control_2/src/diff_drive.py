#!/usr/bin/python3
from std_msgs.msg import Float64
import rospy
import random

def diff_drive(control, linear_vel, angular_vel):
    fl = Float64()
    fr = Float64()
    bl = Float64()
    br = Float64()
    
    l = Float64()
    r = Float64()

    linear_vel = linear_vel * -1 # Because the wheel y axis is -1 in the urdf
    angular_vel = angular_vel * -1 # Because the wheel y axis is -1 in the urdf

    # left_speed = (v - e w) / R 
    # right_speed = (v + e w) / R
    # v is the linear velocity given as parameter
    # w is the angular velocity given as parameter
    # e is half of the distance between the left and right wheels
    # R is the radius of the wheel

    fl.data = (linear_vel - angular_vel * 0.195) / 0.05 # Front Left
    bl.data = fl.data                                   # Back Left

    fr.data = (linear_vel + angular_vel * 0.195) / 0.05 # Front Right
    br.data = br.data                                   # Back Right
    
    control.vel_pub_fl.publish(fl.data)
    control.vel_pub_fr.publish(fr.data)
    control.vel_pub_bl.publish(bl.data)
    control.vel_pub_br.publish(br.data)

    control.vel_pub_l.publish(random.randint(-15,5))
    control.vel_pub_r.publish(random.randint(-15,5))
    
