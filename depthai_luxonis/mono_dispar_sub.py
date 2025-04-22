import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DualImageSubscriber(Node):

    def __init__(self):
        super().__init__('dual_image_subscriber')
        self.bridge = CvBridge()
        self.disparity_subscription = self.create_subscription(
            Image,
            '/oak/disparity',  # Replace with your disparity image topic
            self.disparity_callback,
            10)
        self.monochrome_subscription = self.create_subscription(
            Image,
            '/oak/mono_left',  # Replace with your aligned monochrome image topic
            self.monochrome_callback,
            10)
        self.disparity_image = None
        self.monochrome_image = None

    def disparity_callback(self, msg):
        try:
            self.disparity_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.get_logger().info(f'Received disparity {msg.width}x{msg.height} image')
            self.process_images()
        except Exception as e:
            self.get_logger().error(f'Error converting disparity image: {e}')

    def monochrome_callback(self, msg):
        try:
            self.monochrome_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8') # Adjust encoding if needed
            self.get_logger().info(f'Received monochrome {msg.width}x{msg.height} image')
            self.process_images()
        except Exception as e:
            self.get_logger().error(f'Error converting monochrome image: {e}')

    def process_images(self):
        if self.disparity_image is not None and self.monochrome_image is not None:
            self.get_logger().info('Both images received. Processing...')
            # Perform your processing logic here using self.disparity_image and self.monochrome_image
            self.disparity_image = None  # Reset for the next pair
            self.monochrome_image = None # Reset for the next pair

def main(args=None):
    rclpy.init(args=args)
    dual_image_subscriber = DualImageSubscriber()
    rclpy.spin(dual_image_subscriber)
    dual_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
