#!/usr/bin/env python3

import rospy
from msd700_msg.msg import WebNavCommand
from geometry_msgs.msg import Point, Pose

def test_publish(publisher):
    # Create and initialize the message
    point_msg = WebNavCommand()

    # Initialize pose if it's not already initialized
    point_msg.pose.pose = Pose()  # Make sure the pose is initialized

    # Assign values to the pose fields
    point_msg.command = "A"
    point_msg.pose.pose.position = Point(10, 10, 0)  # Initialize position with x=10, y=10, z=0

    # Publish the message
    publisher.publish(point_msg)

    # Optionally, publish again with the same values
    publisher.publish(point_msg)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('test_custom_pub')

        # Create the publisher (only once)
        publisher = rospy.Publisher('web_point_cmd', WebNavCommand, queue_size=10)

        # Give the publisher some time to connect to subscribers
        rospy.sleep(1)  

        # Test publishing the message
        test_publish(publisher)

        # Keep the program running to allow publishing
        rospy.spin()

    except rospy.ROSInterruptException:
        import sys
        print("Program interrupted before completion", file=sys.stderr)
