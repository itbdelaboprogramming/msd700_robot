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
from msd700_msg.msg import WebNavCommand

cmd_pos = None
x_pos   = None
y_pos   = None
z_pos   = None
w_ort   = None
x_ort   = None
y_ort   = None
z_ort   = None

point_stack = []

def sub_callback(msg: WebNavCommand):
    global cmd_pos, x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort
    cmd_pos = msg.command
    x_pos   = msg.pose.pose.position.x
    y_pos   = msg.pose.pose.position.y
    z_pos   = 0.0
    w_ort   = msg.pose.pose.orientation.w
    x_ort   = 0.0
    y_ort   = 0.0
    z_ort   = msg.pose.pose.orientation.z

command_pos = rospy.Subscriber('web_point_cmd', WebNavCommand, sub_callback, queue_size=1)


def add_point():
    """ Publish the point and store it in an array/stack """
    global cmd_pos, x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort
    point = (x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort)
    
    if point not in point_stack:
        point_stack.append(point)
        print("added new point to stack")
    else:
        print("point not included, duplicated")


def delete_point():
    """ Check the array/stack, delete all points in radius 0.1 """
    global cmd_pos, x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort
    threshold = 0.1
    original_stack = point_stack[:]
    point_stack[:] = [p for p in point_stack if not (abs(p[0] - x_pos) <= threshold and abs(p[1] - y_pos) <= threshold)]
    print(f"Previous stack : {original_stack}")
    print(f"Deleted points near ({x_pos}, {y_pos}). New list: {point_stack}")
    

def clear_point():
    """ Publish the points as new point and store it in an array """
    global cmd_pos, x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort
    point_stack.clear()
    print("cleared all")


def save_point():
    """ read the array/stack and convert to  """
    global cmd_pos, x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort

    filename = "points_data.txt"
    with open(filename, "w") as file:
        for point in point_stack:
            file.write(f"{point}\n")
    print(f"Saved points to {filename}.")
    

if __name__ == '__main__':
    rospy.init_node('test_custom_pos')
    rate = rospy.Rate(100)
    try:
        while not rospy.is_shutdown():
            rospy.sleep(1)  

            if cmd_pos   == "A":
                add_point()
            elif cmd_pos == "D":
                delete_point()
            elif cmd_pos == "X":
                clear_point()
            elif cmd_pos == "S":
                save_point()

            rate.sleep()
            cmd_pos = None

            print(point_stack)
        

    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)


