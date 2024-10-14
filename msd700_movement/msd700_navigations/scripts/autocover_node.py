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

def extract_occupied_positions(grid: OccupancyGrid):
    resolution = grid.info.resolution
    origin_x = grid.info.origin.position.x
    origin_y = grid.info.origin.position.y
    width = grid.info.width
    height = grid.info.height

    occupied_positions = []

    # Loop through the occupancy grid data
    for k, value in enumerate(grid.data):
        if value == 100:  # Only consider occupied cells
            # Convert 1D index `k` to 2D indices (i, j)
            i = k // width
            j = k % width

            # Convert to real-world coordinates
            x = origin_x + j * resolution
            y = origin_y + i * resolution 

            occupied_positions.append((x, y))

    return occupied_positions

def autocover_node():
    map_msg: OccupancyGrid = rospy.wait_for_message("/map", OccupancyGrid)

    occupied_positions = extract_occupied_positions(map_msg)

    min_x = min(occupied_positions, key=lambda x: x[0])[0]
    max_x = max(occupied_positions, key=lambda x: x[0])[0]
    min_y = min(occupied_positions, key=lambda x: x[1])[1]
    max_y = max(occupied_positions, key=lambda x: x[1])[1]
    max_all = max(abs(min_x), abs(max_x), abs(min_y), abs(max_y))

    min_width_map = map_msg.info.origin.position.x
    min_height_map = map_msg.info.origin.position.y
    max_width_map = map_msg.info.origin.position.x + map_msg.info.width * map_msg.info.resolution
    max_height_map = map_msg.info.origin.position.y + map_msg.info.height * map_msg.info.resolution

    upper_left = (max_all, max_all)
    if (upper_left[0] > max_width_map):
        upper_left[0] = max_width_map
    if (upper_left[1] > max_height_map):
        upper_left[1] = max_height_map

    upper_right = (max_all, -max_all)
    if (upper_right[0] > max_width_map):
        upper_right[0] = max_width_map
    if (upper_right[1] < min_height_map):
        upper_right[1] = min_height_map

    lower_left = (-max_all, max_all)
    if (lower_left[0] < min_width_map):
        lower_left[0] = min_width_map
    if (lower_left[1] > max_height_map):
        lower_left[1] = max_height_map

    lower_right = (-max_all, -max_all)
    if (lower_right[0] < min_width_map):
        lower_right[0] = min_width_map
    if (lower_right[1] < min_height_map):
        lower_right[1] = min_height_map

    rate = rospy.Rate(4)

    points = []
    point = Point()
    point.x = lower_left[0]
    point.y = lower_left[1]
    point.z = 0.0
    points.append(deepcopy(point))
    
    point = Point()
    point.x = lower_left[0]
    point.y = lower_left[1]
    point.z = 0.0
    points.append(deepcopy(point))

    point = Point()
    point.x = upper_left[0]
    point.y = upper_left[1]
    point.z = 0.0
    points.append(deepcopy(point))

    point = Point()
    point.x = upper_right[0]
    point.y = upper_right[1]
    point.z = 0.0
    points.append(deepcopy(point))

    point = Point()
    point.x = lower_right[0]
    point.y = lower_right[1]
    point.z = 0.0
    points.append(deepcopy(point))

    point = Point()
    point.x = lower_left[0]
    point.y = lower_left[1]
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