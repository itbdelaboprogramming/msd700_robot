#!/usr/bin/env python3

import pygame as py
import sys
import rospy  

from geometry_msgs.msg import Twist
# from msd700_msg.msg import Twist

global linear_x, linear_y, linear_z
global angular_x, angular_y, angular_z

# Declare Variables
message = {0,0}

def get_keypressed():

    # Pygame init
    py.init()
    display = py.display.set_mode((300, 300))
    py.display.set_caption("Key Press Detection")

    # Setup Publisher 
    rospy.init_node('im_a_controller', anonymous=True)


    teleop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    teleop_msg = Twist()
    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        for event in py.event.get():
            if event.type == py.QUIT:
                py.quit()
                sys.exit()
        keys = py.key.get_pressed()
        # key_pressed = None

        # Check each defined key and update key_pressed if any is pressed
        # The velocity is in m/s
        if keys[py.K_w]:
            linear_x    = 0.345
            linear_y    = 0
            linear_z    = 0
            angular_x   = 0
            angular_y   = 0
            angular_z   = 0
            
        elif keys[py.K_s]:
            linear_x    = -0.345
            linear_y    = 0
            linear_z    = 0
            angular_x   = 0
            angular_y   = 0
            angular_z   = 0
            
        elif keys[py.K_d]:
            linear_x    = 0
            linear_y    = 0
            linear_z    = 0
            angular_x   = 0
            angular_y   = 0
            angular_z   = -0.5
            
        elif keys[py.K_a]:
            linear_x    = 0
            linear_y    = 0
            linear_z    = 0
            angular_x   = 0
            angular_y   = 0
            angular_z   = 0.5
            
        else:
            linear_x    = 0
            linear_y    = 0
            linear_z    = 0
            angular_x   = 0
            angular_y   = 0
            angular_z   = 0

        py.display.update()

        teleop_msg.linear.x     = linear_x
        teleop_msg.linear.y     = linear_y
        teleop_msg.linear.z     = linear_z
        teleop_msg.angular.x    = angular_x
        teleop_msg.angular.y    = angular_y
        teleop_msg.angular.z    = angular_z

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
