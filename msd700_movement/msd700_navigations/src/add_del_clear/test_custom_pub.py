#!/usr/bin/env python3

import rospy
from msd700_msg.msg import WebNavCommand
from geometry_msgs.msg import Twist

def test_publish(publisher):
    
    point_msg   = WebNavCommand()

    # point_msg.command   = "A"
    # point_msg.pose.pose.position.x = -1
    # point_msg.pose.pose.position.y = 1
    # point_msg.pose.pose.position.z = 0
    # point_msg.pose.pose.orientation.w = 1
    # point_msg.pose.pose.orientation.x = 0
    # point_msg.pose.pose.orientation.y = 0
    # point_msg.pose.pose.orientation.z = 0
    # publisher.publish(point_msg)
    # rospy.sleep(1)  
    
    # point_msg.command   = "A"
    # point_msg.pose.pose.position.x = -1
    # point_msg.pose.pose.position.y = 3
    # point_msg.pose.pose.position.z = 0
    # point_msg.pose.pose.orientation.w = 1
    # point_msg.pose.pose.orientation.x = 0
    # point_msg.pose.pose.orientation.y = 0
    # point_msg.pose.pose.orientation.z = 0
    # publisher.publish(point_msg)
    # rospy.sleep(1)  

    point_msg.command   = "P"
    point_msg.pose.pose.position.x = 0
    point_msg.pose.pose.position.y = 0
    point_msg.pose.pose.position.z = 0
    point_msg.pose.pose.orientation.w = 0
    point_msg.pose.pose.orientation.x = 0
    point_msg.pose.pose.orientation.y = 0
    point_msg.pose.pose.orientation.z = 0
    publisher.publish(point_msg)

def interrupt_pause(publisher):
    cmd_vel_msg = Twist()

    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0
    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = 0
    publisher.publish(cmd_vel_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('test_custom_pub')
        publisher   = rospy.Publisher('web_point_cmd', WebNavCommand, queue_size=10)    
        cmd_pub     = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
        rate = rospy.Rate(1000)

        # rospy.sleep(1)  
        # test_publish(publisher)
        # rospy.spin()
        while not rospy.is_shutdown():
            rospy.sleep(1)  
            test_publish(publisher)
            # interrupt_pause(cmd_pub)
            rate.sleep()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)


