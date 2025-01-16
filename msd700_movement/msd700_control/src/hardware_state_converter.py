#!/usr/bin/env python3

# Import Python Libraries
import rospy
from msd700_msgs.msg import HardwareState
# include your robot's message types                    <<<====================== Important


hardware_state_pub = rospy.Publisher('hardware_state', HardwareState, queue_size=1)

def robot_state_callback(msg: {"YOUR_MSGS_TYPE"}):
    global right_motor_pulse_delta, left_motor_pulse_delta, roll, pitch, yaw, acc_x, acc_y, acc_z, \
    gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z, az_offset, sub_count, accum_yaw, curr_yaw, last_yaw, delta_yaw


    # <<<================================= Important
    right_motor_pulse_delta     = msg.right_motor_pulse_delta   # Here are the example, Change according your published variable 
    left_motor_pulse_delta      = msg.left_motor_pulse_delta    # Here are the example, Change according your published variable 
    roll, pitch, yaw            = 0,0,0         # Change according your published r,p,y variable, use "msg.XXXX"
    acc_x, acc_y, acc_z         = 0,0,0         # Change according your published accelerometer variable, use "msg.XXXX"
    gyr_x, gyr_y, gyr_z         = 0,0,0         # Change according your published gyroscope variable, use "msg.XXXX"
    mag_x, mag_y, mag_z         = 0,0,0         # Change according your published magnetometer variable, use "msg.XXXX"
robot_state_sub = rospy.Subscriber("{YOUR_TOPIC}", {"YOUR_MSGS_TYPE"}, robot_state_callback) # Change based on your topic
# hardware_state_sub = rospy.Subscriber("hardware_state", HardwareState, robot_state_callback) # example


try:    
    while not rospy.is_shutdown():
        hadware_msg = HardwareState()
    
        #Assign odometry msg
        hadware_msg.right_motor_pulse_delta = right_motor_pulse_delta
        hadware_msg.left_motor_pulse_delta  = left_motor_pulse_delta
        hadware_msg.heading = yaw
        hadware_msg.pitch   = pitch
        hadware_msg.roll    = roll
        hadware_msg.acc_x   = acc_x
        hadware_msg.acc_y   = acc_y
        hadware_msg.acc_z   = acc_z
        hadware_msg.gyr_x   = gyr_x
        hadware_msg.gyr_y   = gyr_y
        hadware_msg.gyr_z   = gyr_z
        hadware_msg.mag_x   = mag_x
        hadware_msg.mag_y   = mag_y
        hadware_msg.mag_z   = mag_z
        hardware_state_pub.publish(hadware_msg)
        
        rate.sleep()
        
        
except BaseException as e:
    print(e)
    pass
