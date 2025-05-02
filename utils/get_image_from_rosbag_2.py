import rosbag2_py
import rclpy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from PIL import Image as PILImage
import os

def extract_interactive_frames(bag_path, image_topic):
    """
    Plays an image topic from a ROS 2 bag (.mcap format) and allows saving frames
    by pressing a key. Subsequent saved frames are numbered.

    Args:
        bag_path (str): Path to the ROS 2 bag folder (containing .mcap and .yaml).
        image_topic (str): Name of the image topic in the bag.
    """
    rclpy.init()
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'))
    except Exception as e:
        print(f"Error opening bag file: {e}")
        rclpy.shutdown()
        return

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
    saved_count = 0
    base_filename = os.path.splitext(os.path.basename(bag_path))[0] if os.path.isdir(bag_path) else "frame"

    print("Playing video stream. Press 's' to save the current frame, 'q' to quit.")

    while rclpy.ok() and reader.has_next():
        try:
            (topic, data, timestamp) = reader.read_next()
            if topic == image_topic:
                img_msg = Image()
                img_msg.deserialize(data)

                height = img_msg.height
                width = img_msg.width
                step = img_msg.step
                encoding = img_msg.encoding
                image_data = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(height, step)

                if encoding == "rgb8":
                    cv_image = cv2.cvtColor(image_data[:, :width * 3], cv2.COLOR_RGB2BGR)
                elif encoding == "bgr8":
                    cv_image = image_data[:, :width * 3]
                elif encoding == "mono8":
                    cv_image = cv2.cvtColor(image_data[:, :width], cv2.COLOR_GRAY2BGR)
                else:
                    print(f"Warning: Unsupported image encoding '{encoding}'. Skipping display for this frame.")
                    continue

                cv2.imshow("Video Stream", cv_image)
                key = cv2.waitKey(1) & 0xFF  # Wait for a short time and get the pressed key

                if key == ord('s'):
                    output_filename = f"{base_filename}_frame_{saved_count:04d}.jpg"
                    pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
                    pil_image.save(output_filename, "JPEG")
                    print(f"Saved frame as '{output_filename}'")
                    saved_count += 1
                elif key == ord('q'):
                    break

                frame_count += 1
        except Exception as e:
            print(f"Error processing frame: {e}")
            break

    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    bag_folder = 'your_bag_folder'  # Replace with the actual path to your bag folder
    camera_topic = '/camera/image_raw'  # Replace with the actual image topic name

    extract_interactive_frames(bag_folder, camera_topic)
