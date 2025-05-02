import rosbag2_py
import rclpy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from io import BytesIO
from PIL import Image as PILImage  # Using PIL for JPEG encoding

def save_frame_from_rosbag(bag_path, image_topic, output_filename, frame_number=0):
    """
    Extracts a specific frame from an image topic in a ROS 2 bag and saves it as a JPG.

    Args:
        bag_path (str): Path to the ROS 2 bag file.
        image_topic (str): Name of the image topic in the bag.
        output_filename (str): Name of the file to save the frame as (e.g., "frame.jpg").
        frame_number (int): The index of the frame to extract (default is the first frame).
                           Set to None to extract the last frame.
    """
    rclpy.init()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'))

    topic_types = reader.get_all_topics_and_types()
    image_topic_info = None
    for topic_name, topic_type in topic_types:
        if topic_name == image_topic:
            image_topic_info = topic_type
            break

    if not image_topic_info:
        print(f"Error: Image topic '{image_topic}' not found in the bag.")
        rclpy.shutdown()
        return

    reader.set_filter(rosbag2_py.StorageFilter(topics=[image_topic]))

    frame_count = 0
    target_frame = frame_number
    last_frame_data = None

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == image_topic:
            img_msg = Image()
            img_msg.deserialize(data)

            # Directly use numpy and OpenCV to process the image data
            height = img_msg.height
            width = img_msg.width
            step = img_msg.step
            encoding = img_msg.encoding
            image_data = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(height, step)

            # Handle different encodings (assuming common ones)
            if encoding == "rgb8":
                cv_image = cv2.cvtColor(image_data[:, :width * 3], cv2.COLOR_RGB2BGR)
            elif encoding == "bgr8":
                cv_image = image_data[:, :width * 3]
            elif encoding == "mono8":
                cv_image = cv2.cvtColor(image_data[:, :width], cv2.COLOR_GRAY2BGR)
            else:
                print(f"Warning: Unsupported image encoding '{encoding}'. Skipping frame.")
                continue

            if target_frame is None:
                last_frame_data = cv_image
            elif frame_count == target_frame:
                # Save the OpenCV image directly as JPG using PIL
                pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
                pil_image.save(output_filename, "JPEG")
                print(f"Saved frame {frame_count} to '{output_filename}'")
                rclpy.shutdown()
                return

            frame_count += 1

    if target_frame is None and last_frame_data is not None:
        pil_image = PILImage.fromarray(cv2.cvtColor(last_frame_data, cv2.COLOR_BGR2RGB))
        pil_image.save(output_filename, "JPEG")
        print(f"Saved the last frame to '{output_filename}'")
    elif target_frame is not None and frame_count <= target_frame:
        print(f"Error: Frame number {target_frame} not found in the bag on topic '{image_topic}'.")

    rclpy.shutdown()

if __name__ == '__main__':
    bag_file = 'your_bag_file.db3'  # Replace with the actual path to your bag file
    camera_topic = '/camera/image_raw'  # Replace with the actual image topic name
    output_file = 'extracted_frame.jpg'

    # To save the first frame (index 0):
    save_frame_from_rosbag(bag_file, camera_topic, output_file)

    # To save a specific frame (e.g., the 100th frame):
    # save_frame_from_rosbag(bag_file, camera_topic, 'frame_100.jpg', frame_number=99)

    # To save the last frame:
    # save_frame_from_rosbag(bag_file, camera_topic, 'last_frame.jpg', frame_number=None)
