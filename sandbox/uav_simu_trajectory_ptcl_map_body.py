# same as uav_trajectory_pub , unfinished plot of trajectory and add a point in the map frame.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, TransformStamped, Point
from tf2_ros import TransformBroadcaster
import numpy as np
import time
import math

class AnimatedUAVPublisher(Node):
    def __init__(self):
        super().__init__('animated_uav_publisher')

        # Define the start and end points for the UAV's trajectory
        self.start_position = np.array([1.0, -0.5, 0.2])
        self.end_position = np.array([5.0, 2.0, 1.5])
        self.total_duration = 10.0  # seconds for the animation
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.animation_rate = 30.0  # Hz
        self.trajectory_points = []

        # Define the fixed point in the map frame
        self.map_fixed_point = [2.0, 3.0, 1.0]

        # Publisher for the point cloud in the body frame
        self.cloud_publisher = self.create_publisher(
            PointCloud2, 'body_cloud', 10
        )

        # Publisher for the point in the map frame (static)
        self.map_point_publisher = self.create_publisher(
            PointStamped, 'map_point', 10
        )

        # Publisher for the UAV's trajectory points
        self.trajectory_publisher = self.create_publisher(
            PointStamped, 'uav_trajectory', 10
        )

        # Transform broadcaster to publish the body frame's pose in the map frame
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.animation_rate, self.animation_step)

        self.get_logger().info('UAV animation publisher started.')

    def generate_point_cloud(self):
        points = np.random.rand(100, 3).astype(np.float32) - 0.5
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'body'
        cloud_msg = pc2.create_cloud(header, fields, points)
        return cloud_msg

    def publish_map_point(self):
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'
        point_msg.point.x = self.map_fixed_point[0]
        point_msg.point.y = self.map_fixed_point[1]
        point_msg.point.z = self.map_fixed_point[2]
        self.map_point_publisher.publish(point_msg)

    def publish_trajectory_point(self, position):
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'
        point_msg.point.x = position[0]
        point_msg.point.y = position[1]
        point_msg.point.z = position[2]
        self.trajectory_publisher.publish(point_msg)

    def publish_body_transform(self, current_position, roll=0.0, pitch=0.0, yaw=0.0):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'body'
        t.transform.translation.x = current_position[0]
        t.transform.translation.y = current_position[1]
        t.transform.translation.z = current_position[2]

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

        self.tf_broadcaster.sendTransform(t)

    def animation_step(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        if elapsed_time < self.total_duration:
            fraction = elapsed_time / self.total_duration
            current_position = (1 - fraction) * self.start_position + fraction * self.end_position
            self.publish_body_transform(current_position, yaw=fraction * 2 * math.pi)
            self.cloud_publisher.publish(self.generate_point_cloud())
            self.publish_map_point()
            self.publish_trajectory_point(current_position) # Publish the current position as a trajectory point
        else:
            self.destroy_timer(self.timer)
            self.get_logger().info('UAV animation finished.')

def main(args=None):
    rclpy.init(args=args)
    node = AnimatedUAVPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
