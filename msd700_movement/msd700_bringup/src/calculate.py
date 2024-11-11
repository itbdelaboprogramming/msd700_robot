#!/usr/bin/env python3

import rospy
import numpy as np
from ros_msd700_msgs.msg import HardwareState


rospy.init_node('speed_calculator')

t0 = rospy.Time.now()
t1 = rospy.Time.now()
right_motor_pulse_delta = 0
left_motor_pulse_delta = 0
wheel_radius        = rospy.get_param("/raw_sensor/wheel_radius", 2.75)	       
wheel_distance      = rospy.get_param("/raw_sensor/wheel_distance", 23.0)       
gear_ratio          = rospy.get_param("/raw_sensor/gear_ratio", 1980.0)

def hardware_state_callback(msg: HardwareState):
    global right_motor_pulse_delta, left_motor_pulse_delta, t1
    
    right_motor_pulse_delta = msg.right_motor_pulse_delta
    left_motor_pulse_delta  = msg.left_motor_pulse_delta
    t1 = rospy.Time.now()
hardware_state_sub = rospy.Subscriber("hardware_state", HardwareState, hardware_state_callback)

def calculate_speed(pulse_delta, time):
    distance    = pulse_delta/gear_ratio * 2 * np.pi * wheel_radius/100
    speed       = distance/time
    return speed 

try:
    while not rospy.is_shutdown():
        if t1 != t0: 
            delta_t = (t1 - t0).to_sec()
            
            right_speed     = calculate_speed(right_motor_pulse_delta, delta_t)
            left_speed      = calculate_speed(left_motor_pulse_delta, delta_t)
            
            

            rospy.loginfo(f"Right wheel speed: {right_speed:.3f} m/s, Left wheel speed: {left_speed:.3f} m/s")
            
            # Update t0 to the latest time t1
            t0 = t1

except rospy.ROSInterruptException as e:
    rospy.logerr(f"Error: {e}")