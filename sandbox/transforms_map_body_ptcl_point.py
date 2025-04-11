# run with rviz -d tf_mapbody.rviz
# to show the body frame, rviz already has a display type "TF"
# make sure to increase the scale (all should be setup in the rviz config file)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header  # Correct import for Header
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import time
import math

class PointCloudAndMapPointPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_and_map_point_publisher')

        # Define the fixed point in the map frame
        self.map_fixed_point = [2.0, 3.0, 1.0]  # Example coordinates in the map frame

        # Define the initial pose of the body frame in the map frame
        self.body_x = 1.0
        self.body_y = -0.5
        self.body_z = 0.2
        self.body_roll = 0.0
        self.body_pitch = 0.0
        self.body_yaw = 0.785  # 45 degrees in radians

        # Publisher for the point cloud in the body frame
        self.cloud_publisher = self.create_publisher(
            PointCloud2, 'body_cloud', 10
        )

        # Publisher for the point in the map frame
        self.map_point_publisher = self.create_publisher(
            PointStamped, 'map_point', 10
        )

        # Transform broadcaster to publish the body frame's pose in the map frame
        self.tf_broadcaster = TransformBroadcaster(self)

        self.publish_cloud()  # Publish an initial cloud
        self.publish_map_point() # Publish an initial map point
        self.publish_body_transform()

        # Timer to periodically update and publish data (optional, for dynamic scenarios)
        self.timer = self.create_timer(1.0, self.publish_all)

    def publish_cloud(self):
        # Create a simple point cloud (replace with your actual data)
        points = np.array([[0.1, 0.2, 0.3],
                           [-0.1, 0.1, 0.4],
                           [0.0, -0.2, 0.2]], dtype=np.float32)
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        header = Header()  # Create a Header object
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'body'
        cloud_msg = pc2.create_cloud(header, fields, points)
        self.cloud_publisher.publish(cloud_msg)
        self.get_logger().info('Published PointCloud2 in the body frame')

    def publish_map_point(self):
        # Create a PointStamped message for the fixed point in the map frame
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'
        point_msg.point.x = self.map_fixed_point[0]
        point_msg.point.y = self.map_fixed_point[1]
        point_msg.point.z = self.map_fixed_point[2]
        self.map_point_publisher.publish(point_msg)
        self.get_logger().info(f'Published PointStamped in the map frame: {self.map_fixed_point}')

    def publish_body_transform(self):
        # Create a TransformStamped message defining the transformation from map to body
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'body'
        t.transform.translation.x = self.body_x
        t.transform.translation.y = self.body_y
        t.transform.translation.z = self.body_z

        # Convert Euler angles (roll, pitch, yaw) to quaternion
        roll = self.body_roll
        pitch = self.body_pitch
        yaw = self.body_yaw

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q_w = cr * cp * cy + sr * sp * sy
        q_x = sr * cp * cy - cr * sp * sy
        q_y = cr * sp * cy + sr * cp * sy
        q_z = cr * cp * sy - sr * sp * cy

        t.transform.rotation.x = q_x
        t.transform.rotation.y = q_y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)  # Corrected method name

        self.get_logger().info('Published map to body transform')

    def publish_all(self):
        self.publish_cloud()
        self.publish_map_point()
        self.publish_body_transform()

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAndMapPointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
