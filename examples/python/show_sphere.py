#!/usr/bin/env python3
# code from:
# https://docs.hello-robot.com/0.3/ros2/example_4/

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class Balloon(Node):
    def __init__(self):
        #super().__init__('show_sphere')
        super().__init__('show_sphere_rviz_py')
        #super().__init__('stretch_marker')
        self.publisher_ = self.create_publisher(Marker, 'show_sphere', 10)  
        #self.publisher_ = self.create_publisher(Marker, 'balloon', 10)  

        self.marker = Marker()
        self.marker.header.frame_id = '/base_link'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = self.marker.SPHERE
        self.marker.type = self.marker.SPHERE_LIST
        self.marker.id = 0
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 2.0
        self.get_logger().info("Publishing the show_sphere topic. Use RViz to visualize.")
        #self.get_logger().info("Publishing the balloon topic. Use RViz to visualize.")
    def publish_marker(self):
        self.publisher_.publish(self.marker)
def main(args=None):
    rclpy.init(args=args)
    #balloon = Balloon()
    show_sphere = Balloon()
    while rclpy.ok():
        show_sphere.publish_marker()
    show_sphere.destroy_node()  
    rclpy.shutdown()
if __name__ == '__main__':
    main()
