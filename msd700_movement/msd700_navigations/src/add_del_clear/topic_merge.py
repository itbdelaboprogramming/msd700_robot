#!/usr/bin/env python3

#=============================================================
# This code receive msg from 2D Nav Goal 
# merge the PoseStamped msg with command from command generator
# Publish the merged msg to topic web_point_cmd


# Created       : 17 November 2024
# Maintainer    : 


# Program workflow
# - Read the Key from pygame window (A,D,X,S,P)
# - Subscribe to the topic from 2D Nav Goal
# - Merge those reading
# - Publish the merged reading to "web_point_cmd" topic if new msg exist
# - Give logic P to start

#  source  "https://wiki.ros.org/move_base"
#=============================================================


import pygame as py
import rospy
import sys
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from msd700_msg.msg import WebNavCommand

key_pressed = None
key_past    = None    
x_pos, y_pos, z_pos         = None, None, None
w_ort, x_ort, y_ort, z_ort  = None, None, None, None


point_pub = rospy.Publisher('web_point_cmd', WebNavCommand, queue_size=10)

def get_keypressed():
    global key_pressed

    # Define colors
    white = (255, 255, 255)
    black = (0, 0, 0)
    gray = (200, 200, 200)
    blue = (0, 0, 255)
    blink_color = (100, 100, 100)  # Color for the blink effect

    # Font and button dimensions
    font = py.font.Font('freesansbold.ttf', 24)
    button_width, button_height = 120, 50

    # Button labels and positions
    buttons = {
        "Add": (90, 10),
        "Delete": (90, 70),
        "Clear": (90, 130),
        "Save": (90, 190),
        "Play": (90, 250),
        "Hold": (90, 310),
        "Resume": (90, 370),
        "Stop": (90, 430),
        "Mode 1": (300, 10),
        "Mode 2": (300, 70),
    }

    # Function to draw buttons
    def draw_buttons(highlight_label=None):
        display.fill(white)  # Clear the screen with a white background
        for label, pos in buttons.items():
            color = blink_color if label == highlight_label else gray
            py.draw.rect(display, color, (pos[0], pos[1], button_width, button_height))
            text = font.render(label, True, black)
            text_rect = text.get_rect(center=(pos[0] + button_width // 2, pos[1] + button_height // 2))
            display.blit(text, text_rect)
        py.display.update()

    draw_buttons()  # Initial button drawing

    for event in py.event.get():
        if event.type == py.QUIT:
            py.quit()
            sys.exit()

        # Check for mouse click on buttons
        if event.type == py.MOUSEBUTTONDOWN:
            mouse_pos = event.pos
            for label, pos in buttons.items():
                button_rect = py.Rect(pos[0], pos[1], button_width, button_height)
                if button_rect.collidepoint(mouse_pos):
                    # Blink the button
                    draw_buttons(highlight_label=label)
                    py.time.delay(200)  # Pause for 200ms for the blink effect
                    draw_buttons()  # Redraw buttons to reset the color

                    # Set key_pressed based on button clicked
                    if label == "Add":
                        key_pressed = "A"
                    elif label == "Delete":
                        key_pressed = "D"
                    elif label == "Clear":
                        key_pressed = "X"
                    elif label == "Save":
                        key_pressed = "S"
                    elif label == "Play":
                        key_pressed = "P"
                    elif label == "Hold":
                        key_pressed = "H"
                    elif label == "Resume":
                        key_pressed = "R"
                    elif label == "Stop":
                        key_pressed = "Z"
                    elif label == "Mode 1":
                        key_pressed = "M1"
                    elif label == "Mode 2":
                        key_pressed = "M2"
                    print(f"Button clicked: {label}, key_pressed set to {key_pressed}")
                    return  # Exit after processing the button click

        # Check for keypresses
        if event.type == py.KEYDOWN:
            if event.key == py.K_a:
                key_pressed = "A"
            elif event.key == py.K_d:
                key_pressed = "D"
            elif event.key == py.K_x:
                key_pressed = "X"
            elif event.key == py.K_s:
                key_pressed = "S"
            elif event.key == py.K_p:
                key_pressed = "P"
            elif event.key == py.K_h:
                key_pressed = "H"
            elif event.key == py.K_r:
                key_pressed = "R"
            elif event.key == py.K_z:
                key_pressed = "Z"

            if key_pressed:
                print(f"Key pressed: {key_pressed}")
                return

def pose_callback(msg : PoseStamped):
    global x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort
    x_pos = msg.pose.position.x
    y_pos = msg.pose.position.y
    z_pos = msg.pose.position.z
    w_ort = msg.pose.orientation.w
    x_ort = msg.pose.orientation.x
    y_ort = msg.pose.orientation.y
    z_ort = msg.pose.orientation.z

def point_callback(msg : PointStamped):
    global x_pos, y_pos, z_pos, w_ort, x_ort, y_ort, z_ort
    x_pos = msg.point.x
    y_pos = msg.point.y
    z_pos = msg.point.z
point_sub = rospy.Subscriber('/clicked_point', PointStamped, point_callback, queue_size=10)

def publish_msg(publisher):
    point_msg = WebNavCommand()
    point_msg.command                   = key_pressed
    point_msg.pose.pose.position.x      = x_pos
    point_msg.pose.pose.position.y      = y_pos
    point_msg.pose.pose.position.z      = z_pos
    point_msg.pose.pose.orientation.w   = 1.0
    # point_msg.pose.pose.orientation.w   = w_ort
    # point_msg.pose.pose.orientation.x   = x_ort
    # point_msg.pose.pose.orientation.y   = y_ort
    # point_msg.pose.pose.orientation.z   = z_ort
    
    publisher.publish(point_msg)
    print(point_msg)

def publish_instant(publisher):
    point_msg = WebNavCommand()
    point_msg.command                   = key_pressed
    point_msg.pose.pose.position.x      = 0.0
    point_msg.pose.pose.position.y      = 0.0
    point_msg.pose.pose.position.z      = 0.0
    point_msg.pose.pose.orientation.w   = 1.0

    publisher.publish(point_msg)
    print(f"published : {point_msg}")

def emergency_pub(publisher):
    point_msg = Twist()
    point_msg.linear                   = (0.0, 0.0, 0.0)
    point_msg.angular                  = (0.0, 0.0, 0.0)

    publisher.publish(point_msg)
    print(f"published : {point_msg}")

if __name__ == '__main__':
    try:
        # Initialization
        rospy.init_node('topic_merge')
        rate = rospy.Rate(100)

        py.init()
        display = py.display.set_mode((510,500))

        # Setup Subscriber and Publisher
        # point_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, pose_callback, queue_size=10)

        while not rospy.is_shutdown():
            get_keypressed()

            if (key_pressed == "M1") or (key_pressed == "M2")  or (key_pressed == "P") or (key_pressed == "H") or (key_pressed == "Z") or (key_pressed == "X") or (key_pressed == "R"):
                if key_past is not key_pressed:
                    publish_instant(point_pub)
            if (key_pressed == "Z"):
                if key_past is not key_pressed:
                    publish_instant(point_pub)
            elif (x_pos is not None) or (y_pos is not None) or (z_pos is not None):
                publish_msg(point_pub)

            # Reset Value
            key_past = key_pressed
            x_pos, y_pos, z_pos = None, None, None
            w_ort, x_ort, y_ort, z_ort = None, None, None, None

            rate.sleep()

    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)


