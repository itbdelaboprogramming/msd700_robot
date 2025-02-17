import rospy
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def point_in_polygon(x, y, poly):
    """
    Fungsi untuk mengecek apakah titik (x, y) berada di dalam polygon.
    Menggunakan algoritma ray-casting untuk point in polygon.
    """
    n = len(poly)
    inside = False
    p1x, p1y = poly[0]
    for i in range(n + 1):
        p2x, p2y = poly[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

class LaserFilterNode(object):
    def __init__(self):
        rospy.init_node('laser_filter_node', anonymous=True)

        # Get nested parameters
        params = rospy.get_param('/laser_filter', None)
        if params is None:
            rospy.logerr("Parameter '/laser_filter' not loaded. Check your YAML file and launch file.")
            rospy.signal_shutdown("Missing parameters.")
            return
        
        # Print parameter values
        rospy.loginfo("Node Configuration with the following parameters:")
        rospy.loginfo("point_1: (%f, %f)", params.get('point_1_x', 0.0), params.get('point_1_y', 0.0))
        rospy.loginfo("point_2: (%f, %f)", params.get('point_2_x', 0.0), params.get('point_2_y', 0.0))
        rospy.loginfo("point_3: (%f, %f)", params.get('point_3_x', 0.0), params.get('point_3_y', 0.0))
        rospy.loginfo("point_4: (%f, %f)", params.get('point_4_x', 0.0), params.get('point_4_y', 0.0))
        rospy.loginfo("display_rviz: %s", params.get('display_rviz', True))
        rospy.loginfo("topic_unfiltered: %s", params.get('topic_unfiltered', '/laser_unfiltered'))
        rospy.loginfo("topic_filtered: %s", params.get('topic_filtered', '/laser_filtered'))

        # Membaca parameter dari file YAML
        self.point1 = (params.get('point_1_x', 0.0), params.get('point_1_y', 0.0))
        self.point2 = (params.get('point_2_x', 0.0), params.get('point_2_y', 0.0))
        self.point3 = (params.get('point_3_x', 0.0), params.get('point_3_y', 0.0))
        self.point4 = (params.get('point_4_x', 0.0), params.get('point_4_y', 0.0))
        self.display_rviz = params.get('display_rviz', True)
        self.topic_unfiltered = params.get('topic_unfiltered', '/laser_unfiltered')
        self.topic_filtered = params.get('topic_filtered', '/laser_filtered')

        # Definisikan area filter sebagai list titik (polygon)
        self.polygon = [self.point1, self.point2, self.point3, self.point4]

        # Setup publisher dan subscriber
        self.scan_pub = rospy.Publisher(self.topic_filtered, LaserScan, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.topic_unfiltered, LaserScan, self.scan_callback)

        if self.display_rviz:
            self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        rospy.loginfo("Laser filter node telah diinisialisasi.")

    def scan_callback(self, scan_msg):
        """
        Callback untuk pesan LaserScan.
        Setiap titik dalam scan dikonversi dari polar ke Cartesian dan dicek apakah berada di dalam area filter.
        Jika iya, maka nilainya diset menjadi infinity untuk menandakan filter.
        """
        # Buat pesan LaserScan baru untuk data yang telah difilter
        filtered_scan = LaserScan()
        filtered_scan.header = scan_msg.header
        filtered_scan.angle_min = scan_msg.angle_min
        filtered_scan.angle_max = scan_msg.angle_max
        filtered_scan.angle_increment = scan_msg.angle_increment
        filtered_scan.time_increment = scan_msg.time_increment
        filtered_scan.scan_time = scan_msg.scan_time
        filtered_scan.range_min = scan_msg.range_min
        filtered_scan.range_max = scan_msg.range_max

        filtered_ranges = list(scan_msg.ranges)
        for i, r in enumerate(scan_msg.ranges):
            # Lewati jika jarak tidak valid
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue

            # Konversi ke koordinat Cartesian
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # Jika titik berada di dalam area filter, set nilai menjadi infinity
            if point_in_polygon(x, y, self.polygon):
                filtered_ranges[i] = float('inf')

        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = scan_msg.intensities  # jika data intensitas tersedia

        self.scan_pub.publish(filtered_scan)

        if self.display_rviz:
            self.publish_marker(scan_msg.header)

    def publish_marker(self, header):
        """
        Publikasikan marker RViz untuk menggambarkan area filter.
        Marker berupa LINE_STRIP yang menghubungkan keempat titik dan mengulangi titik awal untuk menutup loop.
        """
        marker = Marker()
        marker.header = header
        marker.ns = "laser_filter"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Lebar garis
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Tambahkan titik-titik dari polygon dan ulangi titik pertama untuk menutup loop
        for pt in self.polygon:
            p = Point()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            marker.points.append(p)
        p = Point()
        p.x = self.polygon[0][0]
        p.y = self.polygon[0][1]
        p.z = 0.0
        marker.points.append(p)

        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        node = LaserFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
