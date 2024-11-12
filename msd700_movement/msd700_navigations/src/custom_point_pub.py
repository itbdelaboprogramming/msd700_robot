#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped

# Declare global variables to hold the message data
x_sub = None
y_sub = None
z_sub = None

my_topic = "/test_topic"  # Change according to your topic
msg_type = PointStamped  # Change according to your topic

rospy.init_node('web_to_rviz_pub', anonymous=True)


def converter_callback(msg: msg_type):
    global x_sub, y_sub, z_sub

    # Update the values when a new message is received
    x_sub = msg.point.x
    y_sub = msg.point.y
    z_sub = msg.point.z

    # Publish the received data immediately after receiving it
    publisher = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)

    point_msg = PointStamped()

    # Assign point message values
    point_msg.header.frame_id = "map"
    # point_msg.header.stamp = rospy.Time.now()  # Use simulation time
    point_msg.point.x = x_sub
    point_msg.point.y = y_sub
    point_msg.point.z = z_sub
    publisher.publish(point_msg)


if __name__ == '__main__':
    try:
        # Subscriber to the topic, calls converter_callback on message reception
        converter_sub = rospy.Subscriber(my_topic, msg_type, converter_callback)

        # Keep the node alive and wait for messages
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
