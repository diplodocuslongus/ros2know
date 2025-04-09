import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from your_package_name.msg import PolarHistogram  # Replace with your custom message

import numpy as np

class PolarHistogramPublisher(Node):
    def __init__(self):
        super().__init__('polar_histogram_publisher')
        self.publisher_ = self.create_publisher(PolarHistogram, 'polar_histogram', 10)
        self.timer = self.create_timer(1.0, self.publish_histogram)

    def publish_histogram(self):
        msg = PolarHistogram()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # Or your sensor frame

        # Example histogram data (replace with your actual calculation)
        num_bins = 36
        angles = np.linspace(0, 2 * np.pi, num_bins, endpoint=False).tolist()
        magnitudes = np.random.rand(num_bins).tolist()  # Example random magnitudes

        msg.angles = angles
        msg.magnitudes = magnitudes

        self.publisher_.publish(msg)
        self.get_logger().info('Published polar histogram')

def main(args=None):
    rclpy.init(args=args)
    polar_histogram_publisher = PolarHistogramPublisher()
    rclpy.spin(polar_histogram_publisher)
    polar_histogram_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
