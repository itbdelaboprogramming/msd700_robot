#!/usr/bin/env python3

import rospy  
from msd700_msg.msg import HardwareCommand
import getch


# Set global variable
right_motor_speed = 0
left_motor_speed = 0


def keys():
    global right_motor_speed, left_motor_speed
    
    rospy.init_node('im_a_controller', anonymous=True)
    teleop_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)
    teleop_msg = HardwareCommand()
    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        key_press   = getch.getch()
        # key_press     = ord(getch.getch())

        if key_press == " ":
            right_motor_speed   = 0    
            left_motor_speed    = 0
        else:
            if key_press == "w":
                right_motor_speed   = 100
                left_motor_speed    = 100

            elif key_press == "s":
                right_motor_speed   = -100
                left_motor_speed    = -100  
        
            elif key_press == "d":
                right_motor_speed   = 0
                left_motor_speed    = 100
        
            elif key_press == "a":
                right_motor_speed   = 100
                left_motor_speed    = 0

            elif key_press == "q":
                right_motor_speed   = 100
                left_motor_speed    = -100
        
            elif key_press == "e":
                right_motor_speed   = -100
                left_motor_speed    = 100

            else :
                right_motor_speed   = 0
                left_motor_speed    = 0
        
        
        teleop_msg.right_motor_speed    = right_motor_speed
        teleop_msg.left_motor_speed     = left_motor_speed

        teleop_pub.publish(teleop_msg)
        rate.sleep()



if __name__ == '__main__':
    try:
        while(1):
            keys()
    except rospy.ROSInterruptException:
        pass