#!/usr/bin/python3
import sys, select, termios, tty
import rospy
from std_msgs.msg import Float64
from diff_drive import diff_drive


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
    
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')

    class robot:
        pass
    robot.vel_pub_fl = rospy.Publisher('/robot5/wheel_fl_velocity_controller/command', Float64, queue_size=1)
    robot.vel_pub_fr = rospy.Publisher('/robot5/wheel_fr_velocity_controller/command', Float64, queue_size=1)
    robot.vel_pub_bl = rospy.Publisher('/robot5/wheel_bl_velocity_controller/command', Float64, queue_size=1)
    robot.vel_pub_br = rospy.Publisher('/robot5/wheel_br_velocity_controller/command', Float64, queue_size=1)
    try:
        linear_vel, angular_vel = 0, 0
        while True:
            key = getKey()
            if key == '\x03':
                break
            if key == 'w':
                linear_vel = 1
            elif key == 's':
                linear_vel = -1
            elif key == 'a':
                angular_vel = 1
            elif key == 'd':
                angular_vel = -1
            elif key == 'q':
                linear_vel, angular_vel = 0, 0
            diff_drive(robot, linear_vel, angular_vel)

    except Exception as e:
        print(e)

    finally:
        diff_drive(robot, 0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)