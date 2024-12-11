#!/usr/bin/env python3

#=============================================================
# This Code Provide Adding, Deleting, Clearing, and Saving 
# the published points as an extension of multiple path planner

# Created       : 12 November 2024
# Maintainer    : 


# Program workflow
# - Subscribe to topic with command and points
# - read the command
#     A : add
#     D : delete
#     X : clear
#     S : save
# - Save points in (csv?)



#=============================================================

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped  
from visualization_msgs.msg import Marker 
from msd700_msg.msg import WebNavCommand

mode    = "looping"
paused  = False  
command = None
x_pos   = None
y_pos   = None
z_pos   = None
w_ort   = None
x_ort   = None
y_ort   = None
z_ort   = None
mode    = None
loop_index = 0
forward = True

point_stack = []
published_stack = []
publish_flag = True

def sub_callback(msg: WebNavCommand):
    global command, x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort, publish_flag, loop_index, forward
    command = msg.command
    x_pos   = msg.pose.pose.position.x
    y_pos   = msg.pose.pose.position.y
    z_pos   = 0.0
    w_ort   = msg.pose.pose.orientation.w
    x_ort   = 0.0
    y_ort   = 0.0
    z_ort   = msg.pose.pose.orientation.z

    if command == "H":
        publish_flag = False
        command = "P"
        print(f"flag toggled to : {publish_flag}")
    elif command == "R":
        publish_flag = True
        command = "P"
        print(f"flag toggled to : {publish_flag}")
    elif command == "P":
        publish_flag = True
        loop_index = 0
        forward = True  
        print(f"loop index set : {loop_index}")

command_pos = rospy.Subscriber('web_point_cmd', WebNavCommand, sub_callback, queue_size=1)

def publish_marker(point, action):
    """Publish a visualization marker"""
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "points"
    marker.id = hash(point) % 1000  # Generate a unique ID based on the point
    marker.type = Marker.SPHERE
    marker.action = action

    # Set position for the marker
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0

    # Define marker properties
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    # Publish the marker
    marker_pub.publish(marker)

def clear_markers():
    """Clear all markers using the DELETEALL action"""
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "points"
    marker.action = Marker.DELETEALL
    marker_pub.publish(marker)

def add_point():
    """Publish the point and store it in an array/stack"""
    global command, x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort
    point = (x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort)
    
    if point not in point_stack:
        point_stack.append(point)
        publish_marker(point, action=Marker.ADD)
        print("Added new point to stack")
    else:
        print("Point not included, duplicated")

def delete_point():
    """Check the array/stack, delete all points in radius 0.1"""
    global command, x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort
    threshold = 0.1
    original_stack = point_stack[:]
    removed_points = []

    # Remove points within the threshold and collect them for marker deletion
    point_stack[:] = [
        p for p in point_stack
        if not (abs(p[0] - x_pos) <= threshold and abs(p[1] - y_pos) <= threshold)
        or removed_points.append(p)
    ]

    # Delete markers for removed points
    for point in removed_points:
        publish_marker(point, action=Marker.DELETE)
    
    print(f"Deleted points near ({x_pos}, {y_pos}). New list: {point_stack}")

def clear_point():
    """Clear all points and markers"""
    global point_stack
    point_stack.clear()

    # Publish a DELETEALL marker action
    clear_markers()
    print("Cleared all points and markers")

def save_point():
    """ read the array/stack and convert to  """
    filename = "points_data.txt"
    with open(filename, "w") as file:
        for point in point_stack:
            file.write(f"{point}\n")
    print(f"Saved points to {filename}.")
    
def publish_goal(goal_point):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set goal position from the stored point
    goal.target_pose.pose.position.x = goal_point[0]
    goal.target_pose.pose.position.y = goal_point[1]
    goal.target_pose.pose.position.z = goal_point[2]
    goal.target_pose.pose.orientation.w = goal_point[3]
    goal.target_pose.pose.orientation.x = goal_point[4]
    goal.target_pose.pose.orientation.y = goal_point[5]
    goal.target_pose.pose.orientation.z = goal_point[6]

    # Send goal to move_base and wait for result
    goal_client.send_goal(goal)
    print(f"Sent goal: {goal_point[:3]}")

    # Wait for the result
    goal_client.wait_for_result()
    result = goal_client.get_result()

    if result:
        print(f"Goal reached: {goal_point[:3]}")
    else:
        print("Failed to reach the goal")

if __name__ == '__main__':
    rospy.init_node('custom_pos')
    rate = rospy.Rate(10)

    goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    goal_client.wait_for_server()

    marker_pub  = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    paused = False  # Variable to track the pause state

    try:
        while not rospy.is_shutdown():
            rospy.sleep(1.0) 
            if command == "A":
                add_point()
            elif command == "D":
                delete_point()
            elif command == "X":
                clear_point()
            elif command == "S":
                save_point()
            elif command == "M1":
                mode = "M1"
            elif command == "M2":
                mode = "M2"
            elif command == "P":
                published_stack = point_stack
                if len(published_stack) > 0:
                    print("Publishing goals in a continuous sequence...")
                    
                    while publish_flag:  # Infinite loop
                        point = published_stack[loop_index]
                        # Publish the goal
                        publish_goal(point)
                        rospy.sleep(5.0)

                        if mode is not None:
                            if mode == "M1": # Mode 12321
                                # Update the index for the next point
                                if forward:
                                    loop_index += 1
                                    if loop_index == len(published_stack) - 1:
                                        forward = False  # Reverse direction
                                else:
                                    loop_index -= 1
                                    if loop_index == 0:
                                        forward = True  # Forward direction again

                                print(f"published index {loop_index}")
                            elif mode == "M2": # Mode 123123
                                forward = True
                                loop_index += 1
                                if loop_index == len(published_stack):
                                    loop_index = 0 # Reset the loop index
                        else:
                            print("Mode hasn't been chosen")
                else:
                    print("No points stored yet.")
            else:
                print("Waiting for a valid command...")
            rate.sleep()

    except rospy.ROSInterruptException:
        print("Program interrupted before completion.")