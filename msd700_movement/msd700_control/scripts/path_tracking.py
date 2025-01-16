import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Empty, EmptyResponse
from collections import deque

class RobotPathTracker:
    def __init__(self):
        rospy.init_node('robot_path_tracker')
        
        # Get nested parameters
        params = rospy.get_param('/robot_path_tracker', None)
        if params is None:
            rospy.logerr("Parameter '/robot_path_tracker' not loaded. Check your YAML file and launch file.")
            rospy.signal_shutdown("Missing parameters.")
            return

        # Parameters
        self.mode_continuous = rospy.get_param('~mode_continuous', True)
        self.max_path = rospy.get_param('~max_path', 100)
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.pose_topic = rospy.get_param('~pose_topic', '/robot_pose')
        self.path_topic = rospy.get_param('~path_topic', '/robot_path')
        self.clear_service_name = rospy.get_param('~clear_service_name', '/clear_path')
        
        rospy.loginfo(f"mode_continuous: {self.mode_continuous}")
        rospy.loginfo(f"max_path: {self.max_path}")
        rospy.loginfo(f"frame_id: {self.frame_id}")
        rospy.loginfo(f"pose_topic: {self.pose_topic}")
        rospy.loginfo(f"path_topic: {self.path_topic}")
        rospy.loginfo(f"clear_service_name: {self.clear_service_name}")

        # Subscribers and Publishers
        self.pose_sub = rospy.Subscriber(self.pose_topic, Pose, self.pose_callback)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)

        # Service
        self.clear_service = rospy.Service(self.clear_service_name, Empty, self.clear_path)

        # Path and Queue
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.path_queue = deque(maxlen=self.max_path if not self.mode_continuous else None)
        self.last_pose = None

    def pose_callback(self, pose_msg):
        # Create a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose = pose_msg

        if self.mode_continuous:
            self.path.poses.append(pose_stamped)
        else:
            # Only add new pose if it differs from the last pose
            # debuggin is pose different
            rospy.loginfo(f"Last pose: {self.last_pose}")
            rospy.loginfo(f"Current pose: {pose_msg}")
            rospy.loginfo(f"Is pose different: {self.is_pose_different(self.last_pose, pose_msg)}")
            
            if self.last_pose is None or self.is_pose_different(self.last_pose, pose_msg):
                if len(self.path_queue) >= self.max_path:
                    self.path_queue.popleft()
                self.path_queue.append(pose_stamped)
                self.last_pose = pose_msg

                # Update path with limited poses
                self.path.poses = list(self.path_queue)
                
                # Print info for debugging
                rospy.loginfo(f"Path length: {len(self.path.poses)}")

        self.path.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path)

    def is_pose_different(self, pose1, pose2):
        try:
            # Tolerance for position and orientation differences
            tolerance_position = 0.01
            tolerance_orientation = 0.01

            # Compare position with tolerance
            position_diff = (abs(pose1.position.x - pose2.position.x) > tolerance_position or
                             abs(pose1.position.y - pose2.position.y) > tolerance_position or
                             abs(pose1.position.z - pose2.position.z) > tolerance_position)

            # Compare orientation with tolerance
            orientation_diff = (abs(pose1.orientation.x - pose2.orientation.x) > tolerance_orientation or
                                abs(pose1.orientation.y - pose2.orientation.y) > tolerance_orientation or
                                abs(pose1.orientation.z - pose2.orientation.z) > tolerance_orientation or
                                abs(pose1.orientation.w - pose2.orientation.w) > tolerance_orientation)

            return position_diff or orientation_diff
        except Exception as e:
            rospy.logerr(f"Error comparing poses: {e}")
            return False

    def clear_path(self, req):
        rospy.loginfo("Clearing path.")
        self.path.poses.clear()
        self.path_queue.clear()
        self.path_pub.publish(self.path)
        return EmptyResponse()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tracker = RobotPathTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
