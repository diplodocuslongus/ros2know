# approach uses CvBridge or not, option to select the QoS profile
# if not using cv_bridge it doesn't show the images with imshow, i don't know why (TODO: why)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
exist_cvbridge = False
try:
    from cv_bridge import CvBridge
    exist_cvbridge = True
except:
    import numpy as np
    print('no cv bridge')

class DualImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        # best_effort_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1, durability=DurabilityPolicy.VOLATILE)
        self.mono_subscription = self.create_subscription(
            Image,
            '/oak/mono_left',
            self.mono_callback,
            10)
            # qos_profile=best_effort_qos)
        self.disparity_subscription = self.create_subscription(
            Image,
            '/oak/disparity',
            self.disparity_callback,
            10)
            # qos_profile=best_effort_qos)
        cv.namedWindow('mono', cv.WINDOW_NORMAL)
        cv.namedWindow('disparity', cv.WINDOW_NORMAL)
        self.disparity_image = None
        self.monochrome_image = None

    def disparity_callback(self, msg):
        try:
            width = msg.width
            height = msg.height
            encoding = msg.encoding
            data = np.frombuffer(msg.data, dtype=np.uint8)  # Convert to NumPy array

            if encoding == "bgr8":
                cv_image = data.reshape((height, width, 3))  # Reshape for BGR
            elif encoding == "rgb8":
                cv_image = data.reshape((height, width, 3))[:, :, ::-1] # Reshape and RGB to BGR
            elif encoding == "mono8":
                cv_image = data.reshape((height, width))  # Reshape for grayscale
            else:
                self.get_logger().warn(f"Unsupported encoding: {encoding}")
                return  

            if cv_image is not None and not cv_image.size == 0: # Check if image is valid
                self.disparity_image = cv_image.copy()
                # cv.imshow('disparity', self.disparity_image)
                self.disparity_msg = msg # Store the ROS message
                self.process_images() # Call the processing function
            else:
                self.get_logger().error("Didn't get disparity.")

        except Exception as e:
            self.get_logger().error(f"Error in disparity_callback: {e}")

    def mono_callback(self, msg):
        try:
            width = msg.width
            height = msg.height
            encoding = msg.encoding
            data = np.frombuffer(msg.data, dtype=np.uint8)  # Convert to NumPy array

            if encoding == "bgr8":
                cv_image = data.reshape((height, width, 3))  # Reshape for BGR
            elif encoding == "rgb8":
                cv_image = data.reshape((height, width, 3))[:, :, ::-1] # Reshape and RGB to BGR
            elif encoding == "mono8":
                cv_image = data.reshape((height, width))  # Reshape for grayscale
            else:
                self.get_logger().warn(f"Unsupported encoding: {encoding}")
                return  

            if cv_image is not None and not cv_image.size == 0: # Check if image is valid
                self.monochrome_image = cv_image.copy()
                cv.imshow('mono', self.monochrome_image)
                self.mono_left_msg = msg # Store the ROS message
                self.process_images() # Call the processing function
        except Exception as e:
            self.get_logger().error(f"Didnt get monoleft: {e}")

    def process_images(self):
        if self.disparity_image is not None and self.monochrome_image is not None:
            self.get_logger().info('Both images received. Processing...')
            self.get_logger().info(f'{self.disparity_image.shape}')
            self.get_logger().info(f'{self.monochrome_image.shape}')
            cv.imshow('disparity', self.disparity_image)
            # Perform your processing logic here using self.disparity_image and self.monochrome_image
            self.disparity_image = None  # Reset for the next pair
            self.monochrome_image = None # Reset for the next pair

class DualImageSubscriber_cvbridge(Node):

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
    if exist_cvbridge:
        dual_image_subscriber = DualImageSubscriber_cvbridge()
    else:
        dual_image_subscriber = DualImageSubscriber()
    rclpy.spin(dual_image_subscriber)
    dual_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
