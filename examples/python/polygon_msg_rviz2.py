# publish a rviz2 visualizable polygon
# run this code and
# rviz2 -d polygon_.rviz
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Point32
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class PolygonMarkerExample(Node):
    def __init__(self):
        super().__init__('polygon_marker_example')
        self.publisher_ = self.create_publisher(Marker, 'my_polygon_marker', 10)
        self.timer_ = self.create_timer(1.0, self.publish_marker)

    def create_square_marker(self):
        """Creates a Marker message representing a square polygon."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_polygons"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime.sec = 0  # Ensure the marker persists

        marker.scale.x = 0.1
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        # Define the vertices directly as Point32 objects and convert to Point
        marker.points = [
            Point(x=float(1.0), y=float(1.0), z=float(0.0)),
            Point(x=float(2.0), y=float(1.0), z=float(0.0)),
            Point(x=float(2.0), y=float(2.0), z=float(0.0)),
            Point(x=float(1.0), y=float(2.0), z=float(0.0)),
            Point(x=float(1.0), y=float(1.0), z=float(0.0)),  # Close the loop
        ]

        return marker

    def create_triangle_marker(self):
        """Creates a Marker message representing a triangle polygon."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_polygons"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime.sec = 0

        marker.scale.x = 0.1
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)

        # Define the vertices directly as Point32 objects and convert to Point
        marker.points = [
            Point(x=float(-1.0), y=float(-1.0), z=float(0.0)),
            Point(x=float(0.0), y=float(-2.0), z=float(0.0)),
            Point(x=float(1.0), y=float(-1.0), z=float(0.0)),
            Point(x=float(-1.0), y=float(-1.0), z=float(0.0)),  # Close the loop
        ]

        return marker

    def publish_marker(self):
        """Publishes Marker messages representing the polygons."""
        square_marker = self.create_square_marker()
        self.get_logger().info('Publishing Square Marker')
        self.publisher_.publish(square_marker)

        triangle_marker = self.create_triangle_marker()
        self.get_logger().info('Publishing Triangle Marker')
        self.publisher_.publish(triangle_marker)

def main(args=None):
    rclpy.init(args=args)
    polygon_marker_example = PolygonMarkerExample()
    rclpy.spin(polygon_marker_example)
    polygon_marker_example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
