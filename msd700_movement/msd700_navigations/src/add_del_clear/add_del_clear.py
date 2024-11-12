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

cmd_sub = None
x_sub   = None
y_sub   = None
z_sub   = None

def sub_callback(msg: WebNavCommand):
    global cmd_sub, x_sub, y_sub, z_sub
    cmd_sub = msg.point.command
    x_sub   = msg.point.x
    y_sub   = msg.point.y
    z_sub   = msg.point.z
self.command_sub = rospy.Subscriber('web_point_cmd', WebNavCommand, self.sub_callback, queue_size=10)


def add_point():
""" Publish the point and store it in an array/stack """


def delete_point():
""" Check the array/stack, delete all points in radius 0.1 """


def clear_point():
""" Publish the points as new point and store it in an array """


def save_point():
""" read the array/stack and convert to  """
    


def main():
    match cmd_sub:
        case "A" : add_point()
        case "D" : delete_point()
        case "X" : clear_point()
        case "S" : save_point()


            
