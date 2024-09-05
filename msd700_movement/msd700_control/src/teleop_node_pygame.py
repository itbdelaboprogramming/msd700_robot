#!/usr/bin/env python3

import pygame as py
import sys
import rospy  
from msd700_msg.msg import HardwareCommand


# Declare Variables
message = {0,0}

def get_keypressed():
    global right_motor_speed, left_motor_speed

    # Pygame init
    py.init()
    display = py.display.set_mode((300, 300))
    py.display.set_caption("Key Press Detection")

    # Setup Publisher 
    rospy.init_node('im_a_controller', anonymous=True)
    teleop_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)
    teleop_msg = HardwareCommand()
    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        for event in py.event.get():
            if event.type == py.QUIT:
                py.quit()
                sys.exit()
        keys = py.key.get_pressed()
        # key_pressed = None

        # Check each defined key and update key_pressed if any is pressed
        if keys[py.K_a]:
            right_motor_speed   = 100    
            left_motor_speed    = -100
            # print("W")
            key_pressed = py.K_a
        elif keys[py.K_s]:
            right_motor_speed   = -100    
            left_motor_speed    = -100
            # print("A")
            key_pressed = py.K_s
        elif keys[py.K_d]:
            right_motor_speed   = -100    
            left_motor_speed    = 100
            # print("S")
            key_pressed = py.K_d
        elif keys[py.K_w]:
            right_motor_speed   = 100    
            left_motor_speed    = 100
            # print("D")
            key_pressed = py.K_w
        else:
            right_motor_speed   = 0    
            left_motor_speed    = 0
            # print("NOTHING")


        py.display.update()

        teleop_msg.right_motor_speed    = right_motor_speed
        teleop_msg.left_motor_speed     = left_motor_speed
        teleop_pub.publish(teleop_msg)
        rate.sleep()


def setup_publisher():
    global right_motor_speed, left_motor_speed
    
    rospy.init_node('im_a_controller', anonymous=True)
    teleop_pub = rospy.Publisher('hardware_command', HardwareCommand, queue_size=1)
    teleop_msg = HardwareCommand()
    rate = rospy.Rate(1000)
    
    while not rospy.is_shutdown():  
        teleop_msg.right_motor_speed    = right_motor_speed
        teleop_msg.left_motor_speed     = left_motor_speed

        teleop_pub.publish(teleop_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        # while(1):
        while not rospy.is_shutdown():
            get_keypressed()
            # setup_publisher()
    except rospy.ROSInterruptException:
        rospy.quit()
        pass
