#!/usr/bin/env python3

import rospy
from msd700_msg.msg import WebNavCommand

def test_publish(publisher):
    
    point_msg   = WebNavCommand()

    # point_msg.command   = "A"
    # point_msg.pose.pose.position.x = 0
    # point_msg.pose.pose.position.y = 9
    # point_msg.pose.pose.position.z = 1

    # publisher.publish(point_msg)

    # point_msg.command   = "A"
    # point_msg.pose.pose.position.x = 12
    # point_msg.pose.pose.position.y = 12
    # point_msg.pose.pose.position.z = 12

    # publisher.publish(point_msg)

    # point_msg.command   = "D"
    # point_msg.pose.pose.position.x = 10
    # point_msg.pose.pose.position.y = 10
    # point_msg.pose.pose.position.z = 10

    point_msg.command   = "X"
    point_msg.pose.pose.position.x = 0
    point_msg.pose.pose.position.y = 0
    point_msg.pose.pose.position.z = 0
    publisher.publish(point_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('test_custom_pub')
        publisher   = rospy.Publisher('web_point_cmd', WebNavCommand, queue_size=10)    
        rate = rospy.Rate(100)

        # rospy.sleep(1)  
        # test_publish(publisher)
        # rospy.spin()
        while not rospy.is_shutdown():
            rospy.sleep(1)  
            test_publish(publisher)
            rate.sleep()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)


