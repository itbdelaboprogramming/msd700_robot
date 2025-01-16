#!/usr/bin/env python3

"""
This code converts cmd_vel to hardware command
1. Subscribe to cmd_vel topic
2. Compute angular velocity for each motor
3. Publish to HardwareCommand


Additional
1. include yaml file to execute params
2. use PID, put PID parameters in yaml
"""


import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from msd700_msgs.msg import HardwareState
from msd700_msgs.msg import HardwareCommand
import numpy as np


# Initialize ROS Node
rospy.init_node('bridger')


# Get parameters
wheel_radius        = rospy.get_param("/raw_sensor/wheel_radius", 2.75)	       
wheel_distance      = rospy.get_param("/raw_sensor/wheel_distance", 100.0)       
gear_ratio          = rospy.get_param("/raw_sensor/gear_ratio", 1980.0)


# Initialization
t0 = rospy.Time.now()
t1 = rospy.Time.now()
right_motor_pulse_delta = 0
left_motor_pulse_delta = 0

vx = 0.0
vy = 0.0
vz = 0.0
wx = 0.0
wy = 0.0
wz = 0.0
target_left = 0.0
target_right = 0.0


# Hardcoded
# mul_factor = 10         # add more speed
# rotation_factor = 10    # add more speed

mul_factor = 1              # add more speed
rotation_factor = 2         # add more speed

# Main loop Setup
rate = rospy.Rate(100)

# Publisher
command_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)
command_msg = HardwareCommand()

# Subscriber
def cmd_vel_callback(msg: Twist):
    global vx, vy, vz, wx, wy, wz
    vx = msg.linear.x       # in m/s
    vy = msg.linear.y       # in m/s
    vz = msg.linear.z       # in m/s
    wx = msg.angular.x
    wy = msg.angular.y
    wz = msg.angular.z
cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

def actual_vel_callback(msg:HardwareState):
    global right_motor_pulse_delta, left_motor_pulse_delta, t1
    
    right_motor_pulse_delta = msg.right_motor_pulse_delta
    left_motor_pulse_delta  = msg.left_motor_pulse_delta
    t1 = rospy.Time.now()
actual_vel_sub = rospy.Subscriber("/hardware_state", HardwareState, actual_vel_callback)
 
def calculate_speed(pulse_delta, time):
    distance    = pulse_delta/gear_ratio * 2 * np.pi * wheel_radius/100
    speed       = distance/time
    return speed 

def convert_rpm():
    global target_right, target_left, right_motor_speed, left_motor_speed
    # target_left    = (vx-wz*wheel_distance/(2*100)*rotation_factor)/(wheel_radius/100) * mul_factor     # in rpm
    # target_right   = (vx+wz*wheel_distance/(2*100)*rotation_factor)/(wheel_radius/100) * mul_factor     # in rpm

    wheel_circumference = 2*np.pi*wheel_radius

    target_left_mps    = vx-wz*wheel_distance/(2*100) * rotation_factor     # in meter per second (mps)
    target_right_mps   = vx+wz*wheel_distance/(2*100) * rotation_factor     # in meter per second (mps)

    target_left_rpm    = 60*(target_left_mps/(wheel_circumference/100))    # in rotation per meter (rpm)
    target_right_rpm   = 60*(target_right_mps/(wheel_circumference/100))    # in rotation per meter (rpm)

    command_msg.right_motor_speed   = target_right_rpm # right_motor_speed #  
    command_msg.left_motor_speed    = target_left_rpm # left_motor_speed  #   
    command_pub.publish(command_msg)
    rate.sleep()


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            convert_rpm()

    except rospy.ROSInterruptException:
        rospy.quit()
        pass
