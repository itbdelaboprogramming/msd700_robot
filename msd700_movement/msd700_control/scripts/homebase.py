import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from msd700_msg.msg import WebNavCommand
import tf2_geometry_msgs
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

class HomeBaseNode:
    def __init__(self):
        rospy.init_node("home_base_node")

        # Parameters
        self.x = rospy.get_param("~x", None)
        self.y = rospy.get_param("~y", None)
        self.home_base_frame = rospy.get_param("~home_base_frame", "map")
        self.odom_frame = rospy.get_param("~odom", "odom")
        self.topic_pub_location = rospy.get_param("~topic_pub_location", "/homebase/location")
        self.topic_sub_sethome = rospy.get_param("~topic_sub_sethome", "/homebase/sethomebase")
        self.service_change_homebase_rviz = rospy.get_param("~service_change_homebase_rviz", "/homebase/service_change_homebase_rviz")
        self.msg_command = rospy.get_param("~msg_command", "H")
        self.rate_publish_homebase = rospy.get_param("~rate_publish_homebase", 10)
        self.service_goto = rospy.get_param("~service_goto", "/homebase/goto")
        self.service_goto_cancel = rospy.get_param("~service_goto_cancel", "/homebase/goto_cancel")

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

    def init_homebase(self):
        if self.x is not None and self.y is not None:
            self.home_base_pose = PoseStamped()
            self.home_base_pose.header.frame_id = self.home_base_frame
            self.home_base_pose.pose.position.x = self.x
            self.home_base_pose.pose.position.y = self.y
            self.home_base_pose.pose.orientation.w = 1.0  # Default orientation
        else:
            try:
                transform = self.tf_buffer.lookup_transform(self.home_base_frame, self.odom_frame, rospy.Time(0), rospy.Duration(3.0))
                self.home_base_pose = PoseStamped()
                self.home_base_pose.header.frame_id = self.home_base_frame
                self.home_base_pose.pose.position.x = transform.transform.translation.x
                self.home_base_pose.pose.position.y = transform.transform.translation.y
                self.home_base_pose.pose.orientation = transform.transform.rotation
                rospy.loginfo("Default home base set from odom frame")
            except Exception as e:
                rospy.logerr(f"Failed to initialize home base from TF: {e}")

    def set_homebase_callback(self, msg):
        if msg.command == self.msg_command:
            self.home_base_pose = msg.pose
            rospy.loginfo("Home base updated via topic")

    def change_homebase_rviz_callback(self, req):
        rospy.loginfo("Listening for goal to set new home base")
        self.listening_for_goal = True
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        return EmptyResponse()

    def rviz_goal_callback(self, msg):
        if self.listening_for_goal:
            self.home_base_pose = msg
            self.listening_for_goal = False
            rospy.loginfo("Home base updated via RViz goal")

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

    def publish_homebase_pose(self):
        rate = rospy.Rate(self.rate_publish_homebase)
        while not rospy.is_shutdown():
            if self.home_base_pose:
                self.home_base_pose.header.stamp = rospy.Time.now()
                self.location_pub.publish(self.home_base_pose)
            rate.sleep()

if __name__ == "__main__":
    try:
        node = HomeBaseNode()
        node.publish_homebase_pose()
    except rospy.ROSInterruptException:
        pass
