#!/usr/bin/env python3
import rospy
import actionlib
import pygame as py
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker 

# Initialize global variables
points = []                 # List to store the goals
goal_client = None          # Action client for move_base
marker_publisher = None     # Publisher for RViz markers

def callback(data):
    """Callback function to store points and publish RViz marker"""
    points.append(data)
    print(f"Point stored: {data.point}")
    print(f"Total points stored: {len(points)}")
    
    publish_marker(data, len(points))  # Publish marker in RViz for each point

def publish_marker(point_stamped, number):
    # Publish the sphere marker
    marker = Marker()
    marker.header.frame_id = point_stamped.header.frame_id
    marker.header.stamp = rospy.Time.now()

    # Marker details
    marker.ns = "points"
    marker.id = number  
    marker.type = Marker.SPHERE  
    marker.action = Marker.ADD
    marker.pose.position = point_stamped.point
    marker.pose.orientation.w = 1.0  
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0 

    # Publish the sphere marker
    marker_publisher.publish(marker)

    # Publish the number above the point as text
    text_marker = Marker()
    text_marker.header.frame_id = point_stamped.header.frame_id
    text_marker.header.stamp = rospy.Time.now()

    # Marker details for the text
    text_marker.ns = "numbers"
    text_marker.id = number + 1000  
    text_marker.type = Marker.TEXT_VIEW_FACING  
    text_marker.action = Marker.ADD
    text_marker.pose.position = point_stamped.point
    text_marker.pose.position.z += 1.0  
    text_marker.pose.orientation.w = 1.0  
    text_marker.scale.z = 0.25  
    text_marker.text = str(number)  
    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0 

    # Publish the text marker
    marker_publisher.publish(text_marker)
    print(f"Published text marker '{number}' at: {point_stamped.point}")

def publish_goal(goal_point):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = goal_point.header.frame_id
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set goal position from the stored point
    goal.target_pose.pose.position = goal_point.point
    goal.target_pose.pose.orientation.w = 1.0  # Default orientation

    # Send goal to move_base and wait for result
    goal_client.send_goal(goal)
    print(f"Sent goal: {goal_point.point}")

    # Wait for the result
    goal_client.wait_for_result()
    result = goal_client.get_result()

    if result:
        print(f"Goal reached: {goal_point.point}")
    else:
        print("Failed to reach the goal")

def point_subscriber():
    global goal_client, marker_publisher

    # Initialize the ROS node
    rospy.init_node('move_base_goal_publisher', anonymous=True)

    # Subscribe to the clicked_point topic
    rospy.Subscriber("clicked_point", PointStamped, callback)

    # Initialize the move_base action client
    goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    goal_client.wait_for_server()

    # Initialize the marker publisher for RViz
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Initialize pygame
    py.init()
    display = py.display.set_mode((300, 300))
    py.display.set_caption("Press P to start, N to stop and clear")

    # Main loop
    print("Press 'P' to start publishing goals to move_base one at a time.")
    while not rospy.is_shutdown():
        for event in py.event.get():
            if event.type == py.QUIT:
                py.quit()
                sys.exit()

        keys = py.key.get_pressed()

        if keys[py.K_p]:
            if len(points) > 0:
                print("Publishing goals one by one...")
                for point in points:
                    publish_goal(point) 
                    rospy.sleep(5.0)

            else:
                print("No points stored yet.")
            rospy.sleep(0.5)

        if keys[py.K_n]:
            points.clear()
            print("All points cleared.")
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        point_subscriber()
    except rospy.ROSInterruptException:
        pass
