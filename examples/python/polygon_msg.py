# example of publishing a polygone message in ros2
# code with the assistance from Gemini 
# the code doesn't show anything in rviz2.
# geometry_msgs/Polygon message itself is not directly visualizable in RViz2 
# using the standard "Add" button and selecting a display type. 
# it only shows up under "unvisualizable topics."
#  RViz2 needs a specific display type to interpret and render the Polygon data.
# to Visualize geometry_msgs/Polygon in RViz2:
# Using the Marker Display Type 
# The visualization_msgs/Marker message is a versatile way to display various geometric shapes . 
# we construct a Marker message that uses the points from your Polygon to define a polygon marker.
# this is done in polygon_msg_rviz2.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

class PolygonExample(Node):
    def __init__(self):
        super().__init__('polygon_example')
        self.publisher_ = self.create_publisher(Polygon, 'my_polygon', 10)
        self.timer_ = self.create_timer(1.0, self.publish_polygon)

    def create_line_polygon(self):
        """Creates a simple line polygon, well, a line, i.e. a degenerated polygon."""
        polygon = Polygon()

        # 2 points for the line segment
        point1 = Point32()
        point1.x = 1.0
        point1.y = 1.0
        point1.z = 0.0
        polygon.points.append(point1)

        point2 = Point32()
        point2.x = 2.0
        point2.y = 1.0
        point2.z = 0.0
        polygon.points.append(point2)

        return polygon

    def create_square_polygon(self):
        """Creates a simple square polygon."""
        polygon = Polygon()

        # Define the vertices of the square
        point1 = Point32()
        point1.x = 1.0
        point1.y = 1.0
        point1.z = 0.0
        polygon.points.append(point1)

        point2 = Point32()
        point2.x = 2.0
        point2.y = 1.0
        point2.z = 0.0
        polygon.points.append(point2)

        point3 = Point32()
        point3.x = 2.0
        point3.y = 2.0
        point3.z = 0.0
        polygon.points.append(point3)

        point4 = Point32()
        point4.x = 1.0
        point4.y = 2.0
        point4.z = 0.0
        polygon.points.append(point4)

        return polygon

    def create_triangle_polygon(self):
        """Creates a simple triangle polygon."""
        polygon = Polygon()

        # Define the vertices of the triangle
        point1 = Point32()
        point1.x = -1.0
        point1.y = -1.0
        point1.z = 0.0
        polygon.points.append(point1)

        point2 = Point32()
        point2.x = 0.0
        point2.y = -2.0
        point2.z = 0.0
        polygon.points.append(point2)

        point3 = Point32()
        point3.x = 1.0
        point3.y = -1.0
        point3.z = 0.0
        polygon.points.append(point3)

        return polygon

    def publish_polygon(self):
        """Publishes different polygon messages periodically."""
        square_polygon = self.create_square_polygon()
        self.get_logger().info('Publishing Square Polygon')
        self.publisher_.publish(square_polygon)

        triangle_polygon = self.create_triangle_polygon()
        self.get_logger().info('Publishing Triangle Polygon')
        self.publisher_.publish(triangle_polygon)

def main(args=None):
    rclpy.init(args=args)
    polygon_example = PolygonExample()
    rclpy.spin(polygon_example)
    polygon_example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
