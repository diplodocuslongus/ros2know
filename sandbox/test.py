import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import time
import math

class UAVPublisher(Node):
    def __init__(self):
        super().__init__('uav_publisher')

        # --- Animation Parameters ---
        self.start_position = np.array([1.0, -0.5, 0.2])
        self.end_position = np.array([5.0, 2.0, 1.5])
        self.total_duration = 20.0  # Increased duration to slow down animation (from 10.0)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.animation_rate = 30.0  # Keep the update rate, but the animation will take longer

        # --- Obstacle Definition ---
        self.obstacle_center_map = np.array([3.0, 0.0, 0.5])
        self.obstacle_radius = 1.0
        self.obstacle_height = 2.0
        self.num_points_around = 50
        self.num_levels = 5
        self.obstacle_points_map = self.generate_obstacle_points()
        self.max_detection_distance = 5.0
        self.field_of_view_angle = math.radians(60.0)

        # --- Publishers ---
        self.cloud_publisher = self.create_publisher(
            PointCloud2, 'body_cloud', 10
        )
        self.trajectory_publisher = self.create_publisher(
            PointStamped, 'uav_trajectory', 10
        )
        self.full_obstacle_publisher = self.create_publisher(  # New publisher for the full obstacle
            PointCloud2, 'full_obstacle_cloud', 10
        )

        # --- TF Broadcaster ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / self.animation_rate, self.animation_step)

        # --- Publish the full obstacle once ---
        self.publish_full_obstacle()

        self.get_logger().info('UAV publisher started.')

    def generate_obstacle_points(self):
        points = []
        for i in range(self.num_levels):
            z = self.obstacle_center_map[2] + i * (self.obstacle_height / (self.num_levels - 1)) if self.num_levels > 1 else self.obstacle_center_map[2]
            for j in range(self.num_points_around):
                theta = 2.0 * math.pi * j / self.num_points_around
                x = self.obstacle_center_map[0] + self.obstacle_radius * math.cos(theta)
                y = self.obstacle_center_map[1] + self.obstacle_radius * math.sin(theta)
                points.append([x, y, z])
        return np.array(points, dtype=np.float32)

    def publish_full_obstacle(self):
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # The full obstacle is in the map frame
        cloud_msg = pc2.create_cloud(header, fields, self.obstacle_points_map)
        self.full_obstacle_publisher.publish(cloud_msg)
        self.get_logger().info('Published full obstacle cloud.')

    def get_current_rotation(self):
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
        return np.array([q_x, q_y, q_z, q_w])

    def transform_map_to_body(self, point_map, quaternion):
        roll = self.body_roll
        pitch = self.body_pitch
        yaw = self.body_yaw
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        cb = math.cos(pitch)
        sb = math.sin(pitch)
        ca = math.cos(roll)
        sa = math.sin(roll)
        R_map_to_body = np.array([
            [cy * cb, cy * sb * sa - sy * ca, cy * sb * ca + sy * sa],
            [sy * cb, sy * sb * sa + cy * ca, sy * sb * ca - cy * sa],
            [-sb,    cb * sa,                cb * ca]
        ])
        rotated_point = np.dot(R_map_to_body.T, point_map - np.array([self.body_x, self.body_y, self.body_z]))
        return rotated_point

    def generate_point_cloud(self):
        current_position_map = np.array([self.body_x, self.body_y, self.body_z])
        current_rotation_quaternion = self.get_current_rotation()

        detected_points_body = []
        for obstacle_point_map in self.obstacle_points_map:
            relative_position_map = obstacle_point_map - current_position_map
            point_body = self.transform_map_to_body(relative_position_map, current_rotation_quaternion)

            distance = np.linalg.norm(point_body)
            if distance < self.max_detection_distance:
                angle_horizontal = math.atan2(point_body[1], point_body[0])
                if abs(angle_horizontal) < self.field_of_view_angle / 2.0:
                    detected_points_body.append(point_body)

        detected_points_np = np.array(detected_points_body, dtype=np.float32)

        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'body'

        if detected_points_np.size > 0:
            cloud_msg = pc2.create_cloud(header, fields, detected_points_np)
            return cloud_msg
        else:
            cloud_msg = PointCloud2()
            cloud_msg.header = header
            cloud_msg.height = 1
            cloud_msg.width = 0
            cloud_msg.fields = fields
            cloud_msg.is_bigendian = False
            cloud_msg.point_step = 12
            cloud_msg.row_step = 0
            cloud_msg.is_dense = True
            cloud_msg.data = b''
            return cloud_msg

    def animation_step(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        if elapsed_time < self.total_duration:
            fraction = elapsed_time / self.total_duration
            current_position = (1 - fraction) * self.start_position + fraction * self.end_position
            self.body_x = current_position[0]
            self.body_y = current_position[1]
            self.body_z = current_position[2]

            self.body_roll = 0.0
            self.body_pitch = 0.0
            self.body_yaw = fraction * 2 * math.pi

            self.publish_body_transform(current_position, self.body_roll, self.body_pitch, self.body_yaw)
            self.cloud_publisher.publish(self.generate_point_cloud())
            self.publish_trajectory_point(current_position)
        else:
            self.destroy_timer(self.timer)
            self.get_logger().info('UAV animation finished.')

def main(args=None):
    rclpy.init(args=args)
    node = UAVPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
