import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import struct
import numpy as np

class PolarHistogramVisualization(Node):
    def __init__(self):
        super().__init__('polar_histogram_visualization')
        self.pc2_publisher_ = self.create_publisher(PointCloud2, 'polar_histogram_pc2', 10)
        self.path_publisher_ = self.create_publisher(Path, 'polar_histogram_lines', 10)
        self.timer = self.create_timer(1.0, self.publish_histogram)

    def publish_histogram(self):
        # Example histogram data
        num_bins = 36
        angles = np.linspace(0, 2 * np.pi, num_bins, endpoint=False)
        magnitudes = np.random.rand(num_bins)

        # Publish PointCloud2
        pc2_msg = PointCloud2()
        pc2_msg.header = Header()
        pc2_msg.header.stamp = self.get_clock().now().to_msg()
        pc2_msg.header.frame_id = "map"

        points = []
        for angle, magnitude in zip(angles, magnitudes):
            x = magnitude * np.cos(angle)
            y = magnitude * np.sin(angle)
            z = 0.0
            points.append([x, y, z])

        pc2_msg.height = 1
        pc2_msg.width = len(points)
        pc2_msg.is_dense = True
        pc2_msg.point_step = 4 * 3
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width
        pc2_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc2_msg.data = bytearray(struct.pack('<' + 'fff' * len(points), *np.array(points).flatten()))
        self.pc2_publisher_.publish(pc2_msg)
        self.get_logger().info('Published PointCloud2')

        # Publish Path for lines
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        origin_pose = PoseStamped()
        origin_pose.header = path_msg.header
        origin_pose.pose.position.x = 0.0
        origin_pose.pose.position.y = 0.0
        origin_pose.pose.position.z = 0.0
        origin_pose.pose.orientation.w = 1.0  # Default orientation

        for point in points:
            point_pose = PoseStamped()
            point_pose.header = path_msg.header
            point_pose.pose.position.x = point[0]
            point_pose.pose.position.y = point[1]
            point_pose.pose.position.z = point[2]
            point_pose.pose.orientation.w = 1.0

            path_msg.poses.append(origin_pose)
            path_msg.poses.append(point_pose)

        self.path_publisher_.publish(path_msg)
        self.get_logger().info('Published Path for lines')

def main(args=None):
    rclpy.init(args=args)
    polar_histogram_visualization = PolarHistogramVisualization()
    rclpy.spin(polar_histogram_visualization)
    polar_histogram_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
