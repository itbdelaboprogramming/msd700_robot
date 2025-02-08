#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from msd700_msg.msg import WebNavCommand
import tkinter as tk
from tkinter import messagebox
import signal
import sys

class WebNavGUI:
    def __init__(self):
        rospy.init_node('nav_gui_node')
        self.rate = rospy.Rate(100)

        self.current_command = None
        self.x_pos = self.y_pos = self.z_pos = None

        # Parameters
        self.topic_point_pub = rospy.get_param("~topic_point_pub", "/nav_gui/point_cmd")
        self.topic_point_sub = rospy.get_param("~topic_point_sub", "/clicked_point")

        # Publisher, Subscriber
        self.point_pub = rospy.Publisher(self.topic_point_pub, WebNavCommand, queue_size=10)
        rospy.Subscriber(self.topic_point_sub, PointStamped, self.point_callback)

        self.root = tk.Tk()
        self.root.title("Web Navigation Control")
        self.root.geometry("400x400")

        self.main_frame = tk.Frame(self.root)
        self.edit_frame = tk.Frame(self.root)

        self.setup_main_buttons()
        self.setup_edit_buttons()

        self.show_main_buttons()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        signal.signal(signal.SIGINT, self.signal_handler)
        self.root.mainloop()

    def setup_main_buttons(self):
        # Mode buttons
        tk.Button(self.main_frame, text="Mode 1", command=lambda: self.set_command("M1"), width=10, height=2).grid(row=0, column=0, padx=10, pady=10)
        tk.Button(self.main_frame, text="Mode 2", command=lambda: self.set_command("M2"), width=10, height=2).grid(row=0, column=1, padx=10, pady=10)

        # Action buttons
        tk.Button(self.main_frame, text="Start", command=lambda: self.set_command("P"), width=20, height=3).grid(row=1, column=0, columnspan=2, padx=10, pady=10)
        tk.Button(self.main_frame, text="Stop", command=lambda: self.set_command("Z"), width=20, height=3).grid(row=2, column=0, columnspan=2, padx=10, pady=10)
        tk.Button(self.main_frame, text="Edit Points", command=self.show_edit_buttons, width=20, height=3).grid(row=3, column=0, columnspan=2, padx=10, pady=10)

    def setup_edit_buttons(self):
        # Edit buttons
        tk.Button(self.edit_frame, text="Import Point", command=lambda: self.set_command("I"), width=20, height=2).grid(row=0, column=0, padx=10, pady=5)
        tk.Button(self.edit_frame, text="Add Point", command=lambda: self.set_command("A"), width=20, height=2).grid(row=1, column=0, padx=10, pady=5)
        tk.Button(self.edit_frame, text="Delete Point", command=lambda: self.set_command("D"), width=20, height=2).grid(row=2, column=0, padx=10, pady=5)
        tk.Button(self.edit_frame, text="Clear Stored Points", command=lambda: self.set_command("X"), width=20, height=2).grid(row=3, column=0, padx=10, pady=5)
        tk.Button(self.edit_frame, text="Save Point", command=lambda: self.set_command("S"), width=20, height=2).grid(row=4, column=0, padx=10, pady=5)
        tk.Button(self.edit_frame, text="Done", command=self.show_main_buttons, width=10, height=1).grid(row=5, column=0, sticky='e', padx=10, pady=20)

    def show_main_buttons(self):
        self.current_command = None
        self.edit_frame.pack_forget()
        self.main_frame.pack(fill='both', expand=True)

    def show_edit_buttons(self):
        self.main_frame.pack_forget()
        self.edit_frame.pack(fill='both', expand=True)

    def set_command(self, command):
        self.current_command = command
        if command not in ["A", "D"]:
            self.publish_command()

    def point_callback(self, msg: PointStamped):
        self.x_pos, self.y_pos, self.z_pos = msg.point.x, msg.point.y, msg.point.z
        if self.current_command in ["A", "D"]:
            self.publish_command()

    def publish_command(self):
        if not self.current_command:
            return

        point_msg = WebNavCommand()
        point_msg.command = self.current_command
        point_msg.pose.pose.position.x = self.x_pos if self.x_pos else 0.0
        point_msg.pose.pose.position.y = self.y_pos if self.y_pos else 0.0
        point_msg.pose.pose.position.z = self.z_pos if self.z_pos else 0.0
        point_msg.pose.pose.orientation.w = 1.0

        self.point_pub.publish(point_msg)
        print(f"Published: {point_msg}")

    def on_close(self):
        if messagebox.askokcancel("Exit", "Do you want to Exit?"):
            self.root.quit()
            self.root.destroy()

    def signal_handler(self, sig, frame):
        print("Ctrl-C detected. Exiting program...")
        self.root.quit()
        self.root.destroy()
        sys.exit(0)

if __name__ == '__main__':
    try:
        WebNavGUI()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
