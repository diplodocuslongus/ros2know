# publish a (non visualizable) polygon message idefined by numpy array (or list)
# make it visualizable by reusing the points that defines the polygon in order to show a line marker
# run this code and rviz2 -d polygon_marker.rviz

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np

class LinePolygonExample(Node):
    def __init__(self):
        super().__init__('line_polygon_example')
        self.polygon_publisher_ = self.create_publisher(Polygon, 'line_polygon', 10)
        self.marker_publisher_ = self.create_publisher(Marker, 'line_marker', 10)
        self.timer_ = self.create_timer(1.0, self.publish_line)

    def define_line_polygon(self, points):
        """Creates a geometry_msgs/Polygon message from a list of two points."""
        polygon_msg = Polygon()
        if len(points) == 2:
            p1 = Point32(x=float(points[0][0]), y=float(points[0][1]), z=float(points[0][2]))
            p2 = Point32(x=float(points[1][0]), y=float(points[1][1]), z=float(points[1][2]))
            polygon_msg.points = [p1, p2]
        else:
            self.get_logger().warn("Input should be a list of two points to define a line polygon.")
        return polygon_msg

    def visualize_line_polygon(self, points):
        """Creates a visualization_msgs/Marker message to display a line."""
        marker_msg = Marker()
        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = "line_visualization"
        marker_msg.id = 0
        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.lifetime.sec = 0

        marker_msg.scale.x = 0.05  # Line width
        marker_msg.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green line

        if len(points) == 2:
            p1 = Point(x=float(points[0][0]), y=float(points[0][1]), z=float(points[0][2]))
            p2 = Point(x=float(points[1][0]), y=float(points[1][1]), z=float(points[1][2]))
            marker_msg.points = [p1, p2]
        else:
            self.get_logger().warn("Input should be a list of two points to visualize a line.")

        return marker_msg

    def publish_line(self):
        """Defines two points, creates and publishes both Polygon and Marker messages."""
        line_points_list = [[0.0, 0.0, 0.0], [2.0, 1.0, 0.0]]
        line_points_numpy = np.array([[0.0, -1.0, 0.0], [2.0, 0.0, 0.0]])

        # Publish the Polygon message (not directly visualizable in standard RViz2)
        polygon_msg_list = self.define_line_polygon(line_points_list)
        self.get_logger().info(f"Publishing Polygon with points: {polygon_msg_list.points}")
        self.polygon_publisher_.publish(polygon_msg_list)

        polygon_msg_numpy = self.define_line_polygon(line_points_numpy.tolist()) # Convert numpy array to list
        self.get_logger().info(f"Publishing Polygon with points (from numpy): {polygon_msg_numpy.points}")
        self.polygon_publisher_.publish(polygon_msg_numpy)

        # Publish the Marker message (visualizable as a line in RViz2)
        marker_msg_list = self.visualize_line_polygon(line_points_list)
        self.get_logger().info(f"Publishing Marker with points: {[p.x for p in marker_msg_list.points]}")
        self.marker_publisher_.publish(marker_msg_list)

        marker_msg_numpy = self.visualize_line_polygon(line_points_numpy)
        self.get_logger().info(f"Publishing Marker with points (from numpy): {[p.x for p in marker_msg_numpy.points]}")
        self.marker_publisher_.publish(marker_msg_numpy)

def main(args=None):
    rclpy.init(args=args)
    line_polygon_example = LinePolygonExample()
    rclpy.spin(line_polygon_example)
    line_polygon_example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
