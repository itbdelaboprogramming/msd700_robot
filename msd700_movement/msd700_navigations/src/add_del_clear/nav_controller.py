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
from geometry_msgs.msg import PointStamped, Twist
from visualization_msgs.msg import Marker
from msd700_msg.msg import WebNavCommand
import csv
import signal
import sys

class NavController:
    def __init__(self):
        rospy.init_node('nav_controller_node')
        self.rate = rospy.Rate(10)

        # Parameters
        self.topic_cmd_sub      = rospy.get_param("~topic_cmd_sub", "/nav_gui/point_cmd")
        self.topic_marker_pub   = rospy.get_param("~topic_marker_pub", "/visualization_marker")
        self.topic_stop_pub     = rospy.get_param("~topic_stop_pub", "/mux/emergency_vel")
        self.save_file_path     = rospy.get_param("~save_file_path", "/home/itbdelabo/catkin_ws/src/msd700_robot/msd700_movement/msd700_navigations/saved_point/points.csv")
        self.import_file_path   = rospy.get_param("~import_file_path", "/home/itbdelabo/catkin_ws/src/msd700_robot/msd700_movement/msd700_navigations/saved_point/points.csv")


        # Initialize variables
        self.command = None
        self.x_pos = self.y_pos = self.z_pos = 0.0
        self.w_ort = 1.0
        self.x_ort = self.y_ort = self.z_ort = 0.0
        self.mode = "M1"
        self.loop_index = 0
        self.forward = True
        self.publish_flag = True
        self.canceled_flag = True

        self.point_arr = []
        self.published_arr = []
        

        # Subscribers and Publishers
        rospy.Subscriber(self.topic_cmd_sub, WebNavCommand, self.sub_callback, queue_size=1)
        self.marker_pub = rospy.Publisher(self.topic_marker_pub , Marker, queue_size=10)
        self.stop_pub   = rospy.Publisher(self.topic_stop_pub , Twist, queue_size=10)

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

        if self.command == "STOP":
            self.publish_flag = False
            self.emergency_stop()
            print(f"Flag toggled to: {self.publish_flag}")
        elif self.command == "PAUSE":
            self.publish_flag = False
            self.pause()
            print(f"Flag toggled to: {self.publish_flag}")
        elif self.command == "START":
            self.publish_flag = True
            self.canceled_flag = False
            print(f"Starting mode {self.mode}")
            print(f"Loop index set: {self.loop_index}")
        elif self.command == "SAVE":
            self.save_point()
        elif self.command == "IMPORT":
            self.import_point()
        elif self.command == "CLEAR":
            self.clear_point()
        elif self.command in ["M1", "M2"]:
            self.mode = self.command
            print(f"Changed mode to {self.mode}")

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
        if point not in self.point_arr:
            self.point_arr.append(point)
            self.publish_marker(point, Marker.ADD)
            print("Added new point to stack")

    def delete_point(self):
        threshold = 0.1
        removed_points = [p for p in self.point_arr if abs(p[0] - self.x_pos) <= threshold and abs(p[1] - self.y_pos) <= threshold]
        self.point_arr = [p for p in self.point_arr if p not in removed_points]
        for point in removed_points:
            self.publish_marker(point, Marker.DELETE)
        print(f"Deleted points near ({self.x_pos}, {self.y_pos}). New list: {self.point_arr}")

    def clear_point(self):
        self.point_arr.clear()
        self.clear_markers()
        print("Cleared all points and markers")

    def save_point(self):
        try:
            with open(self.save_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['index', 'x_pos', 'y_pos', 'z_pos', 'w_ort', 'x_ort', 'y_ort', 'z_ort'])  # CSV header
                for idx, point in enumerate(self.point_arr):
                    writer.writerow([idx, *point])
            print(f"Saved {len(self.point_arr)} points to {self.save_file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save points: {e}")

    def import_point(self):
        try:
            with open(self.import_file_path, mode='r') as file:
                reader = csv.DictReader(file)
                self.point_arr.clear()  
                self.clear_markers()
                
                for row in reader:
                    point = (
                        float(row['x_pos']),
                        float(row['y_pos']),
                        float(row['z_pos']),
                        float(row['w_ort']),
                        float(row['x_ort']),
                        float(row['y_ort']),
                        float(row['z_ort'])
                    )
                    self.point_arr.append(point)
                    self.publish_marker(point, Marker.ADD)  # Re-publish imported markers to visualize
            print(f"Imported {len(self.point_arr)} points from {self.import_file_path}")
        except FileNotFoundError:
            rospy.logwarn(f"File {self.import_file_path} not found. No points imported.")
        except Exception as e:
            rospy.logerr(f"Failed to import points: {e}")

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
        state = self.goal_client.get_state()

        print(f"Goal State : {state}")

        if state == 2:
            print("Goal Canceled")
            self.canceled_flag = True
        elif state == 3:
            print(f"Goal reached: {goal_point[:3]}")
        elif state == 4:
            self.goal_client.cancel_all_goals()
            print("Failed to reach the goal, please move the robot to open area")
        else:
            print("Failed to reach the goal")

    def execute_navigation(self):
        self.published_arr = self.point_arr[:]
        if not self.published_arr:
            print("No points stored yet.")
            return

        print("Publishing goals in a continuous sequence...")

        while self.publish_flag:
            point = self.published_arr[self.loop_index]
            self.publish_goal(point)
            rospy.sleep(0.5)

            if not self.canceled_flag:
                if self.mode == "M1":
                    if self.forward:
                        self.loop_index += 1
                        if self.loop_index == len(self.published_arr) - 1:
                            self.forward = False
                    else:
                        self.loop_index -= 1
                        if self.loop_index == 0:
                            self.forward = True
                elif self.mode == "M2":
                    self.forward = True
                    self.loop_index += 1
                    if self.loop_index == len(self.published_arr):
                        self.loop_index = 0
                else:
                    print("Mode hasn't been chosen")
            print(f"Published point {self.loop_index+1}")

    def emergency_stop(self):
        self.goal_client.cancel_all_goals()
        print("All goals have been canceled.")

        stop_msg = Twist()
        stop_msg.linear.x  = 0
        stop_msg.linear.y  = 0
        stop_msg.linear.z  = 0
        stop_msg.angular.x  = 0
        stop_msg.angular.y  = 0
        stop_msg.angular.z  = 0
        self.loop_index = 0

        self.stop_pub.publish(stop_msg)
        print("Emergency stop issued: Robot halted.")

    def pause(self):
        self.goal_client.cancel_all_goals()

        stop_msg = Twist()
        stop_msg.linear.x  = 0
        stop_msg.linear.y  = 0
        stop_msg.linear.z  = 0
        stop_msg.angular.x  = 0
        stop_msg.angular.y  = 0
        stop_msg.angular.z  = 0

        self.stop_pub.publish(stop_msg)
        print("Robot Movement paused")

    def run(self):
        while not rospy.is_shutdown():
            if self.command == "ADD":
                self.add_point()
            elif self.command == "DEL":
                self.delete_point()
            elif self.command == "START":
                self.execute_navigation()
            self.rate.sleep()

    def signal_handler(self, sig, frame):
        print("Ctrl-C detected. Exiting gracefully...")
        sys.exit(0)

if __name__ == '__main__':
    try:
        NavController()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)