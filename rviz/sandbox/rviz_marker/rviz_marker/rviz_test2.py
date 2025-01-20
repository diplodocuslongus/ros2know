# string node from https://www.theconstruct.ai/create-python-publisher-ros2/


import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker,MarkerArray
import math
from geometry_msgs.msg import Point
from std_msgs.msg import String

topic = 'visu_marker_array'
rclpy.init()
node = rclpy.create_node('sos_publisher')
    # publisher = node.create_publisher(String, 'sos')
publisher = node.create_publisher(MarkerArray,topic,10)

markerArray = MarkerArray()
# markerArray.header.stamp = get_clock().now().to_msg()
count = 0
MARKERS_MAX = 100

while rclpy.ok():
# while not rospy.is_shutdown():

    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = math.cos(count / 50.0)
    marker.pose.position.y = math.cos(count / 40.0) 
    marker.pose.position.z = math.cos(count / 30.0) 
 
    # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
    if(count > MARKERS_MAX):
        markerArray.markers.pop(0)
 
    markerArray.markers.append(marker)
 
    # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1
 
    # Publish the MarkerArray
    publisher.publish(markerArray)
 
    count += 1
    # this part needs rework
    # msg = String()
    # def timer_callback():
    #     msg.data = 'ObiWan Kenobi, please help me. You\'re my only hope'
    #     node.get_logger().info('Publishing sos message: "%s"' % msg.data)
    #     publisher.publish(msg)

    # timer_period = 0.5  # seconds
    # timer = node.create_timer(timer_period, timer_callback)

    # rclpy.spin(node)
    # end: this part needs rework
   # rospy.sleep(0.01) # need find the ros2 equivalent
# rclpy.shutdown()
