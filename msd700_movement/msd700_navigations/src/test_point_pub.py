#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped

def publish_point():
    publisher = rospy.Publisher('/test_topic', PointStamped, queue_size=10)
    point_msg = PointStamped()    

    # Give Gazebo and the ROS system time to set up
    rospy.sleep(1)

    # Assign point message
    point_msg.header.frame_id = "map"
    point_msg.header.stamp = rospy.Time.now()  # Use simulation time
    point_msg.point.x = 2.0
    point_msg.point.y = 2.0
    point_msg.point.z = 0.0

    # Publish the message once
    publisher.publish(point_msg)
    rospy.loginfo("Point published: %s", point_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('test_publish')
        rospy.sleep(1)  # Wait to ensure Gazebo and ROS are ready
        publish_point()  # Publish the point once
        rospy.spin()  # Keep the node alive if needed
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
