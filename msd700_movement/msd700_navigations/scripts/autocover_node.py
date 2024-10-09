#! /usr/bin/env python

import actionlib.simple_action_client
import rospy
import sys
import actionlib
from geometry_msgs.msg import PoseStamped, PointStamped, Point
import move_base_msgs.msg
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math        

def autocover_node():
    map_msg: OccupancyGrid = rospy.wait_for_message("/map", OccupancyGrid)
    resolution = map_msg.info.resolution

    # Map width and height (in number of cells)
    width = map_msg.info.width
    height = map_msg.info.height

    # Origin of the map (bottom-left corner)
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y

    rate = rospy.Rate(4) 

    # Calculate the map boundaries
    boundary_x_min = -3.0
    boundary_y_min = -3.0
    boundary_x_max = 3.0
    boundary_y_max = 3.0

    points = []
    point = Point()
    point.x = boundary_x_min
    point.y = boundary_y_max
    point.z = 0.0
    points.append(deepcopy(point))
    
    point = Point()
    point.x = boundary_x_min
    point.y = boundary_y_max
    point.z = 0.0
    points.append(deepcopy(point))

    point = Point()
    point.x = boundary_x_max
    point.y = boundary_y_max
    point.z = 0.0
    points.append(deepcopy(point))

    point = Point()
    point.x = boundary_x_max
    point.y = boundary_y_min
    point.z = 0.0
    points.append(deepcopy(point))

    point = Point()
    point.x = boundary_x_min
    point.y = boundary_y_min
    point.z = 0.0
    points.append(deepcopy(point))

    point = Point()
    point.x = boundary_x_min
    point.y = boundary_y_max
    point.z = 0.0
    points.append(deepcopy(point))

    point_publisher = rospy.Publisher("/clicked_point", PointStamped, queue_size=1)
    i = 0
    while not rospy.is_shutdown():
        if i == 6:
            break
        point_stamped = PointStamped()
        point_stamped.header.frame_id = "map"
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.point = points[i]
        point_publisher.publish(point_stamped)
        print(point_stamped)
        i += 1
        rate.sleep()
    

if __name__ == '__main__':
    try:
        rospy.init_node('autocover_node')
        autocover_node()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)