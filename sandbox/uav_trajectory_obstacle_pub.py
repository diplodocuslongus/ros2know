# This node will:
#    Publish the point cloud in the UAV's body frame.
#    Publish the UAV's trajectory as PointStamped messages in the map frame.
#    Broadcast the transformation from the map frame to the body frame.
#   show a point cloud as if seen by a fized sensor
# try to simulate what a sensor onboard a uav would see...
# use together with map_sub_show.py (run map_sub_show first)
# slow down the simualtion of increase by changing its duration, keeping the same frame rate.

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

        # Define the start and end points for the UAV's trajectory
        self.start_position = np.array([1.0, -0.5, 0.2])
        self.end_position = np.array([2.0, 2.0, 0.5])
        self.total_duration = 20.0  # seconds for the animation
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.animation_rate = 30.0  # Hz

        # for obstacle and oboard sensor simulation
        # self.obstacle_center_map = np.array([3.0, 0.0, 1.0])  # Example: Center of a sphere
        # self.obstacle_radius = 1.5  # Example: Radius of the sphere
        # self.num_obstacle_points = 1000  # Number of points to define the obstacle
        # self.obstacle_points_map = self.generate_obstacle_points()
        # self.max_detection_distance = 5.0
        # self.field_of_view_angle = math.radians(60.0) # Example: 60 degrees horizontal FOV

        self.obstacle_center_map = np.array([3.0, 0.0, 0.5])  # Center of the base of the cylinder
        self.obstacle_radius = 1.0
        self.obstacle_height = 2.0
        self.num_points_around = 50  # Number of points around the circumference at each level
        self.num_levels = 5         # Number of levels along the height

        self.obstacle_points_map = self.generate_cylinder_obstacle_pts()
        # self.obstacle_points_map = self.generate_obstacle_points()
        self.max_detection_distance = 5.0
        self.field_of_view_angle = math.radians(60.0) # Example: 60 degrees horizontal FOV


        # Publisher for the point cloud in the body frame
        self.cloud_publisher = self.create_publisher(
            PointCloud2, 'body_cloud', 10
        )

        # Publisher for the UAV's trajectory points
        self.trajectory_publisher = self.create_publisher(
            PointStamped, 'uav_trajectory', 10
        )

        self.full_obstacle_publisher = self.create_publisher(  # New publisher for the full obstacle
            PointCloud2, 'full_obstacle_cloud', 10
        )

        # Transform broadcaster to publish the body frame's pose in the map frame
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.animation_rate, self.animation_step)

        # --- Publish the full obstacle once 
        self.publish_full_obstacle()        

        self.get_logger().info('UAV publisher started.')

    def generate_obstacle_points(self):
        # Generate points forming the obstacle in the map frame
        # (e.g., points on the surface of a sphere)
        points = []
        for _ in range(self.num_obstacle_points):
            # ... logic to generate points around self.obstacle_center_map with radius self.obstacle_radius ...
            points.append([x, y, z])
        return np.array(points, dtype=np.float32)

    def generate_cylinder_obstacle_pts(self):
        points = []
        for i in range(self.num_levels):
            z = self.obstacle_center_map[2] + i * (self.obstacle_height / (self.num_levels - 1)) if self.num_levels > 1 else self.obstacle_center_map[2]
            for j in range(self.num_points_around):
                theta = 2.0 * math.pi * j / self.num_points_around
                x = self.obstacle_center_map[0] + self.obstacle_radius * math.cos(theta)
                y = self.obstacle_center_map[1] + self.obstacle_radius * math.sin(theta)
                points.append([x, y, z])
        return np.array(points, dtype=np.float32)


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

        if detected_points_np.size > 0:  # Check if there are any detected points
            cloud_msg = pc2.create_cloud(header, fields, detected_points_np)
            return cloud_msg
        else:
            # Return an empty PointCloud2 message if no points are detected
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

    def generate_point_cloud_(self):
        current_position_map = np.array([self.body_x, self.body_y, self.body_z]) # Get current UAV pose
        current_rotation_quaternion = self.get_current_rotation() # Implement this

        detected_points_body = []
        for obstacle_point_map in self.obstacle_points_map:
            # 1. Transform obstacle point from map frame to body frame (inverse transform)
            relative_position_map = obstacle_point_map - current_position_map
            point_body = self.transform_map_to_body(relative_position_map, current_rotation_quaternion)

            # 2. Simulate sensor visibility (distance and angle)
            distance = np.linalg.norm(point_body)
            if distance < self.max_detection_distance:
                # Check if the point is within the field of view (approximate using angle from forward axis)
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
        cloud_msg = pc2.create_cloud(header, fields, detected_points_np)
        return cloud_msg


    def transform_map_to_body(self, point_map, quaternion):
        # Implement the inverse transformation using the quaternion
        # This involves rotating the vector by the inverse quaternion and then translating
        # by the negative of the UAV's position.
        # You might need to use a library for quaternion operations here for correctness.
        # For a simplified case with only yaw rotation, it's easier.
        roll, pitch, yaw = self.body_roll, self.body_pitch, self.body_yaw # Use current rotation
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        cb = math.cos(pitch)
        sb = math.sin(pitch)
        ca = math.cos(roll)
        sa = math.sin(roll)

        # Rotation matrix (map to body, assuming ZYX Euler angles)
        R = np.array([
            [cy * cb, cy * sb * sa - sy * ca, cy * sb * ca + sy * sa],
            [sy * cb, sy * sb * sa + cy * ca, sy * sb * ca - cy * sa],
            [-sb,    cb * sa,                cb * ca]
        ])

        rotated_point = np.dot(R.T, point_map) # Rotate by the transpose (inverse for rotation matrices)
        translated_point = rotated_point - np.array([self.body_x, self.body_y, self.body_z])
        return translated_point

    def get_current_rotation(self):
        # Return the current quaternion representing the UAV's orientation
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
        # current_time = self.get_clock().now().nanoseconds / 1e9
        # elapsed_time = current_time - self.start_time
        # if elapsed_time < self.total_duration:
        #     fraction = elapsed_time / self.total_duration
        #     current_position = (1 - fraction) * self.start_position + fraction * self.end_position
        #     self.publish_body_transform(current_position, yaw=fraction * 2 * math.pi)
        #     self.cloud_publisher.publish(self.generate_point_cloud())
        #     self.publish_trajectory_point(current_position)
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        if elapsed_time < self.total_duration:
            fraction = elapsed_time / self.total_duration
            current_position = (1 - fraction) * self.start_position + fraction * self.end_position
            self.body_x = current_position[0]
            self.body_y = current_position[1]
            self.body_z = current_position[2]

            # Define how the orientation changes over time (example: constant roll and pitch, continuous yaw)
            self.body_roll = 0.0
            self.body_pitch = 0.0
            # fix the yaw or have the uav spin around z-axis
            self.body_yaw = math.pi / 6 # fraction * 2 * math.pi  # Continuous rotation around Z-axis


            # self.publish_body_transform(current_position, yaw=fraction * 2 * math.pi)
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
