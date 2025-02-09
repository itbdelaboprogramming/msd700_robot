#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from msd700_msg.msg import WebNavCommand
import tf2_geometry_msgs
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
import tkinter as tk
import tf.transformations

class HomeBaseNode:
    def __init__(self):
        rospy.init_node("home_base_node")
        
        # Get nested parameters
        params = rospy.get_param('/homebase', None)
        if params is None:
            rospy.logerr("Parameter '/homebase' not loaded. Check your YAML file and launch file.")
            rospy.signal_shutdown("Missing parameters.")
            return

        # Parameters
        self.x = rospy.get_param("~x", None)
        self.y = rospy.get_param("~y", None)
        self.orientation_deg = rospy.get_param("~orientation", 0.0)  # Orientation in degrees
        self.home_base_frame = rospy.get_param("~home_base_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.topic_pub_location = rospy.get_param("~topic_pub_location", "/homebase/location")
        self.topic_sub_sethome = rospy.get_param("~topic_sub_sethome", "/homebase/sethomebase")
        self.service_change_homebase_rviz = rospy.get_param("~service_change_homebase_rviz", "/homebase/service_change_homebase_rviz")
        self.msg_command = rospy.get_param("~msg_command", "H")
        self.rate_publish_homebase = rospy.get_param("~rate_publish_homebase", 10)
        self.service_goto = rospy.get_param("~service_goto", "/homebase/goto")
        self.service_goto_cancel = rospy.get_param("~service_goto_cancel", "/homebase/goto_cancel")
        self.is_use_gui = rospy.get_param('~is_use_gui', True)
        
        # Convert orientation from degrees to quaternion
        self.orientation_quat = tf.transformations.quaternion_from_euler(0, 0, self.orientation_deg * (3.141592653589793 / 180.0))
        
        # Print all parameters
        rospy.loginfo("Home Base Node initialized with parameters:")
        rospy.loginfo(f"x: {self.x}")
        rospy.loginfo(f"y: {self.y}")
        rospy.loginfo(f"orientation: {self.orientation_deg}")
        rospy.loginfo(f"home_base_frame: {self.home_base_frame}")
        rospy.loginfo(f"base_frame: {self.base_frame}")
        rospy.loginfo(f"topic_pub_location: {self.topic_pub_location}")
        rospy.loginfo(f"topic_sub_sethome: {self.topic_sub_sethome}")
        rospy.loginfo(f"service_change_homebase_rviz: {self.service_change_homebase_rviz}")
        rospy.loginfo(f"msg_command: {self.msg_command}")
        rospy.loginfo(f"rate_publish_homebase: {self.rate_publish_homebase}")
        rospy.loginfo(f"service_goto: {self.service_goto}")
        rospy.loginfo(f"service_goto_cancel: {self.service_goto_cancel}")
        rospy.loginfo(f"is_use_gui: {self.is_use_gui}")

        # Variables
        self.home_base_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.listening_for_goal = False

        # Subscribers, publishers, and services
        rospy.Subscriber(self.topic_sub_sethome, WebNavCommand, self.set_homebase_callback)
        self.location_pub = rospy.Publisher(self.topic_pub_location, PoseStamped, queue_size=10)
        rospy.Service(self.service_change_homebase_rviz, Empty, self.change_homebase_rviz_callback)
        rospy.Service(self.service_goto, Empty, self.goto_homebase_callback)
        rospy.Service(self.service_goto_cancel, Empty, self.cancel_goto_homebase_callback)
        self.move_base_goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)

        # Initialize home base position
        self.init_homebase()

        # Start periodic publishing using rospy.Timer
        self.timer_publish = rospy.Timer(rospy.Duration(1.0 / self.rate_publish_homebase), self.publish_homebase_pose)
        
        if self.is_use_gui:
            self.init_gui()
        

    def init_homebase(self):
        if self.x is not None and self.y is not None:
            self.home_base_pose = PoseStamped()
            self.home_base_pose.header.frame_id = self.home_base_frame
            self.home_base_pose.pose.position.x = self.x
            self.home_base_pose.pose.position.y = self.y
            self.home_base_pose.pose.orientation.x = self.orientation_quat[0]
            self.home_base_pose.pose.orientation.y = self.orientation_quat[1]
            self.home_base_pose.pose.orientation.z = self.orientation_quat[2]
            self.home_base_pose.pose.orientation.w = self.orientation_quat[3]
        else:
            try:
                transform = self.tf_buffer.lookup_transform(self.home_base_frame, self.base_frame, rospy.Time(0), rospy.Duration(3.0))
                self.home_base_pose = PoseStamped()
                self.home_base_pose.header.frame_id = self.home_base_frame
                self.home_base_pose.pose.position.x = transform.transform.translation.x
                self.home_base_pose.pose.position.y = transform.transform.translation.y
                self.home_base_pose.pose.orientation = transform.transform.rotation
                rospy.loginfo("Default home base set from odom frame")
            except Exception as e:
                rospy.logerr(f"Failed to initialize home base from TF: {e}")
        
        rospy.loginfo("Home base initialized:")
        self.loginfo_pose(self.home_base_pose)

    def set_homebase_callback(self, msg):
        if msg.command == self.msg_command:
            self.home_base_pose = msg.pose
            rospy.loginfo("Home base updated via topic to:")
            self.loginfo_pose(self.home_base_pose)

    def change_homebase_rviz_callback(self, req):
        rospy.loginfo("Listening for goal to set new home base")
        self.listening_for_goal = True
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        return EmptyResponse()

    def rviz_goal_callback(self, msg):
        if self.listening_for_goal:
            self.home_base_pose = msg
            self.listening_for_goal = False
            rospy.loginfo("Home base updated via RViz goal to :")
            self.loginfo_pose(self.home_base_pose)

    def goto_homebase_callback(self, req):
        rospy.loginfo("Going to home base")

        goal = MoveBaseActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.home_base_pose.header.frame_id
        goal.goal.target_pose = self.home_base_pose

        self.move_base_goal_pub.publish(goal)
        return EmptyResponse()

    def cancel_goto_homebase_callback(self, req):
        cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)
        rospy.loginfo("Cancelled going to home base")
        return EmptyResponse()

    def publish_homebase_pose(self, event):
        if self.home_base_pose:
            self.home_base_pose.header.stamp = rospy.Time.now()
            self.location_pub.publish(self.home_base_pose)

    def init_gui(self):
        root = tk.Tk()
        root.title("Home Base Control")

        btn_goto = tk.Button(root, text="Go to Home Base", command=self.call_goto_service)
        btn_goto.pack(pady=10)

        btn_cancel = tk.Button(root, text="Cancel Go to Home Base", command=self.call_cancel_service)
        btn_cancel.pack(pady=10)

        btn_change_homebase = tk.Button(root, text="Change Home Base via RViz", command=self.call_change_homebase_service)
        btn_change_homebase.pack(pady=10)

        root.mainloop()

    def call_goto_service(self):
        rospy.wait_for_service(self.service_goto)
        try:
            goto_service = rospy.ServiceProxy(self.service_goto, Empty)
            goto_service()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def call_cancel_service(self):
        rospy.wait_for_service(self.service_goto_cancel)
        try:
            cancel_service = rospy.ServiceProxy(self.service_goto_cancel, Empty)
            cancel_service()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def call_change_homebase_service(self):
        rospy.wait_for_service(self.service_change_homebase_rviz)
        try:
            change_homebase_service = rospy.ServiceProxy(self.service_change_homebase_rviz, Empty)
            change_homebase_service()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def quaternion_to_euler(self, quat):
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2] * (180.0 / 3.141592653589793)  # Convert radians to degrees

    def loginfo_pose(self, pose):
        rospy.loginfo(f"x: {pose.pose.position.x}")
        rospy.loginfo(f"y: {pose.pose.position.y}")
        orientation_deg = self.quaternion_to_euler(pose.pose.orientation)
        rospy.loginfo(f"orientation (degrees): {orientation_deg}")


if __name__ == "__main__":
    try:
        node = HomeBaseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
