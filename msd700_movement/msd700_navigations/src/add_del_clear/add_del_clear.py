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
import signal
import sys

class WebNavController:
    def __init__(self):
        rospy.init_node('custom_pos')
        self.rate = rospy.Rate(10)

        # Initialize variables
        self.command = None
        self.x_pos = self.y_pos = self.z_pos = 0.0
        self.w_ort = 1.0
        self.x_ort = self.y_ort = self.z_ort = 0.0
        self.mode = None
        self.loop_index = 0
        self.forward = True
        self.publish_flag = True

        self.point_stack = []
        self.published_stack = []

        # Subscribers and Publishers
        rospy.Subscriber('/nav_gui/point_cmd', WebNavCommand, self.sub_callback, queue_size=1)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        
        self.goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_client.wait_for_server()

        # Handle SIGINT (Ctrl-C)
        signal.signal(signal.SIGINT, self.signal_handler)

        self.run()

    def sub_callback(self, msg: WebNavCommand):
        self.command = msg.command
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.z_pos = 0.0
        self.w_ort = msg.pose.pose.orientation.w
        self.x_ort = 0.0
        self.y_ort = 0.0
        self.z_ort = msg.pose.pose.orientation.z

        if self.command == "H":
            self.publish_flag = False
            self.command = "P"
            print(f"Flag toggled to: {self.publish_flag}")
        elif self.command == "R":
            self.publish_flag = True
            self.command = "P"
            print(f"Flag toggled to: {self.publish_flag}")
        elif self.command == "P":
            self.publish_flag = True
            self.loop_index = 0
            self.forward = True
            print(f"Loop index set: {self.loop_index}")

    def publish_marker(self, point, action):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = hash(point) % 1000
        marker.type = Marker.SPHERE
        marker.action = action

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

    def clear_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)

    def add_point(self):
        point = (self.x_pos, self.y_pos, self.z_pos, self.w_ort, self.x_ort, self.y_ort, self.z_ort)
        if point not in self.point_stack:
            self.point_stack.append(point)
            self.publish_marker(point, Marker.ADD)
            print("Added new point to stack")
        else:
            print("Point not included, duplicated")

    def delete_point(self):
        threshold = 0.1
        removed_points = [p for p in self.point_stack if abs(p[0] - self.x_pos) <= threshold and abs(p[1] - self.y_pos) <= threshold]
        self.point_stack = [p for p in self.point_stack if p not in removed_points]
        for point in removed_points:
            self.publish_marker(point, Marker.DELETE)
        print(f"Deleted points near ({self.x_pos}, {self.y_pos}). New list: {self.point_stack}")

    def clear_point(self):
        self.point_stack.clear()
        self.clear_markers()
        print("Cleared all points and markers")

    def save_point(self):
        with open("points_data.txt", "w") as file:
            for point in self.point_stack:
                file.write(f"{point}\n")
        print("Saved points to points_data.txt")

    def publish_goal(self, goal_point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = goal_point[0]
        goal.target_pose.pose.position.y = goal_point[1]
        goal.target_pose.pose.position.z = goal_point[2]
        goal.target_pose.pose.orientation.w = goal_point[3]
        goal.target_pose.pose.orientation.x = goal_point[4]
        goal.target_pose.pose.orientation.y = goal_point[5]
        goal.target_pose.pose.orientation.z = goal_point[6]

        self.goal_client.send_goal(goal)
        print(f"Sent goal: {goal_point[:3]}")

        self.goal_client.wait_for_result()
        result = self.goal_client.get_result()

        if result:
            print(f"Goal reached: {goal_point[:3]}")
        else:
            print("Failed to reach the goal")

    def run(self):
        while not rospy.is_shutdown():
            if self.command == "A":
                self.add_point()
            elif self.command == "D":
                self.delete_point()
            elif self.command == "X":
                self.clear_point()
            elif self.command == "S":
                self.save_point()
            elif self.command in ["M1", "M2"]:
                self.mode = self.command
            elif self.command == "P":
                self.execute_navigation()
            self.rate.sleep()

    def execute_navigation(self):
        self.published_stack = self.point_stack[:]
        if not self.published_stack:
            print("No points stored yet.")
            return

        print("Publishing goals in a continuous sequence...")
        while self.publish_flag:
            point = self.published_stack[self.loop_index]
            self.publish_goal(point)
            rospy.sleep(5.0)

            if self.mode == "M1":
                if self.forward:
                    self.loop_index += 1
                    if self.loop_index == len(self.published_stack) - 1:
                        self.forward = False
                else:
                    self.loop_index -= 1
                    if self.loop_index == 0:
                        self.forward = True
            elif self.mode == "M2":
                self.forward = True
                self.loop_index += 1
                if self.loop_index == len(self.published_stack):
                    self.loop_index = 0
            else:
                print("Mode hasn't been chosen")
            print(f"Published index {self.loop_index}")

    def signal_handler(self, sig, frame):
        print("Ctrl-C detected. Exiting gracefully...")
        sys.exit(0)

if __name__ == '__main__':
    try:
        WebNavController()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)