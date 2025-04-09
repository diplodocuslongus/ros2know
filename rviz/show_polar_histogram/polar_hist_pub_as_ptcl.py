import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

class PolarHistogramPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('polar_histogram_pointcloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'polar_histogram_pc2', 10)
        self.timer = self.create_timer(1.0, self.publish_histogram)

    def publish_histogram(self):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # Or your sensor frame

        # Example histogram data
        num_bins = 36
        angles = np.linspace(0, 2 * np.pi, num_bins, endpoint=False)
        magnitudes = np.random.rand(num_bins)

        points = []
        for angle, magnitude in zip(angles, magnitudes):
            x = magnitude * np.cos(angle)
            y = magnitude * np.sin(angle)
            z = 0.0
            points.append([x, y, z])

        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.point_step = 4 * 3  # 3 float32 fields (x, y, z)
        msg.row_step = msg.point_step * msg.width

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.data = bytearray(struct.pack('<' + 'fff' * len(points), *np.array(points).flatten()))

        self.publisher_.publish(msg)
        self.get_logger().info('Published polar histogram as PointCloud2')

def main(args=None):
    rclpy.init(args=args)
    polar_histogram_publisher = PolarHistogramPointCloudPublisher()
    rclpy.spin(polar_histogram_publisher)
    polar_histogram_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
