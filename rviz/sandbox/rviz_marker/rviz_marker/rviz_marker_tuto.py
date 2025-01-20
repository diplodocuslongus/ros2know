#!/usr/bin/env python3
# code losely derived from:
# https://docs.hello-robot.com/0.3/ros2/example_4/
# which in fact is directly derived from the cpp version of the ros example
# here I'm expending to allow different node names and topic names 
# for the arrow style marker 
# topics: balloon, arrow (custom name), balloon_list
# info on class inheritence:
# https://www.reddit.com/r/learnpython/comments/ckszou/python_crash_course_inheritance_typeerror_object/
# notes on quaternion / reminder: icos(θ/2)+sin(θ/2)(ivx​+jvy​+kvz​).
# rotataion of theta around a vector vx, vy,vz:
# q =cos(θ/2)+sin(θ/2)(ivx+jvy+kvz)
# remember the quaternion must be valid (i.e exist) and normalized


import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,PointStamped
from builtin_interfaces.msg import Time
import random # for the publish point tests

class LineList2(Node):
    # def __init__(self,namespc = 'ns1'):
    def __init__(self):
        # define the node name
        super().__init__('rviz_marker_linelist2')
        self.publisher_ = self.create_publisher(Marker, 'line_list2_hey', 10) # name the topic 

    def publish_linelist_pt(self,namespc,pts1,pts2):
        marker = Marker()
        marker.header.frame_id = '/base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.LINE_LIST
        marker.id = 0
        marker.action = marker.ADD
        # marker.ns = "salut" 
        marker.ns = namespc 
        marker.scale.x = 0.1 # only scale x is used for line strip markers, it's the width of the line
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        # create a default line
        p0 = Point(x=pts1[0],y=pts1[1],z=pts1[1])
        p1 = Point(x=pts2[1],y=pts2[1],z=pts2[1])
        marker.points.append(p0)
        marker.points.append(p1)
        self.publisher_.publish(marker)
        
    def publish_linelist_pt2(self,namespc,curid,pts1,pts2):

        self.marker = Marker()
        self.marker.header.frame_id = '/base_link'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = self.marker.LINE_LIST
        self.marker.id = curid
        self.marker.action = self.marker.ADD
        # marker.ns = "salut" 
        self.marker.ns = namespc 
        self.marker.scale.x = 0.1 # only scale x is used for line strip markers, it's the width of the line
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        # create a default line
        p0 = Point(x=pts1[0],y=pts1[1],z=pts1[1])
        p1 = Point(x=pts2[1],y=pts2[1],z=pts2[1])
        self.marker.points.append(p0)
        self.marker.points.append(p1)

    # def publish_linelist(self):
    def publish_linelist(self,namespc):
        marker = Marker()
        marker.header.frame_id = '/base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.LINE_LIST
        marker.id = 0
        marker.action = marker.ADD
        # marker.ns = "salut" 
        marker.ns = namespc 
        marker.scale.x = 0.1 # only scale x is used for line strip markers, it's the width of the line
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        # create a default line
        if namespc == 'myns':
            p0 = Point(x=-2.0,y=2.0,z=-1.0)
            p1 = Point(x=2.0,y=2.0,z=1.0)
        elif namespc == 'myns2':
            p0 = Point(x=2.0,y=-2.0,z=-1.0)
            p1 = Point(x=2.0,y=2.0,z=1.0)
        marker.points.append(p0)
        marker.points.append(p1)
        self.publisher_.publish(marker)

        # self.get_logger().info("LINELIST2!!. ")


    def publish_marker(self):
        self.publisher_.publish(self.marker)

class MyPointStamped(Node):
    # def __init__(self,namespc = 'ns1'):
    def __init__(self):
        # define the node name
        super().__init__('PointStamped')
        # name the topic 
        self.publisher_ = self.create_publisher(PointStamped, 'pub_pointstamped', 10) 
        timer_period = 1.0
        self.timer_ = self.create_timer(timer_period,self.publish_point)
        # self.timer = self.create_timer(timer_period,self.publish_point(1.0,0.0,0.0))
        
    def publish_point(self):
    # def publish_point(self,ptx,pty,ptz):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '/base_link'
        ptx=random.random()
        pty=random.random()
        ptz=random.random()
        msg.point.x = ptx # 1.0
        msg.point.y = pty # 1.0
        msg.point.z = ptz # 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing point stamped {msg.point}')



# this is like a super class to allow us to define different nodes
# for the arrow topic, as well as different arrow topic names

class ArrowNode:
    def __init__(self,nodename,topicname):
        self.arrownode=Node(nodename)
        self.arrowtopicname=topicname

class Arrow(ArrowNode):
    def __init__(self,nodename,topicname):
    #def __init__(self,nodename='test'):
        super().__init__(nodename,topicname)
        self.publisher_ = self.arrownode.create_publisher(Marker, self.arrowtopicname, 10)  
        self.marker = Marker()
        self.marker.header.frame_id = '/base_link'
        self.marker.header.stamp = self.arrownode.get_clock().now().to_msg()
        self.marker.type = self.marker.ARROW
        self.marker.id = 0
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 1.5 # arrow length
        self.marker.scale.y = 0.1 # arrow "width"
        self.marker.scale.z = 0.1 # arrow "height"
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        #p = Point()
        #p.x,p.y,p.z = 0.0,0.0,0.0
        #self.marker.points.append(p)
        #p.x,p.y,p.z = 1.0,1.0,1.0
        #self.marker.points.append(p)
        self.arrownode.get_logger().info("Publishing the arrow topic. Use RViz to visualize.")

    def publish_marker(self):
        self.publisher_.publish(self.marker)

    def pos_orient(self,pospt,orient):
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position = pospt

    def start_end(self,startpt,endpt):
        self.marker.scale.x = 1.0 # arrow shaft diameter
        self.marker.scale.y = 1.1 # arrow head diameter
        self.marker.scale.z = 0.1 # arrow head "height"
        self.marker.points.append(startpt)
        self.marker.points.append(endpt)


class LineList(Node):
    def __init__(self,namespc = 'ns1'):
    # def __init__(self):
        # define the node name
        super().__init__('rviz_marker_linelist')
        self.publisher_ = self.create_publisher(Marker, 'line_list_hey', 10) # name the topic 

        self.marker = Marker()
        self.marker.header.frame_id = '/base_link'
        #self.marker.header.frame_id = '/map'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = self.marker.LINE_LIST
        self.marker.id = 0
        self.marker.action = self.marker.ADD
        # self.marker.ns = "salut" 
        self.marker.ns = namespc 
        self.marker.scale.x = 0.1 # only scale x is used for line strip markers, it's the width of the line
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 1.0
        self.marker.pose.position.z = 1.0
        # create a default line
        p0 = Point(x=-2.0,y=2.0,z=0.0)
        p1 = Point(x=-2.0,y=2.0,z=1.0)
        self.marker.points.append(p0)
        self.marker.points.append(p1)
        self.get_logger().info("Publishing the line_strip topic. ")

    def add_linestrip(self,namespc):
        # self.marker = Marker()
        # self.marker.header.frame_id = '/base_link'
        # self.marker.header.stamp = self.get_clock().now().to_msg()
        # self.marker.type = self.marker.LINE_LIST
        # self.marker.id = 0
        # self.marker.action = self.marker.ADD
        self.marker.ns = namespc 
        self.marker.scale.x = 0.1 
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 1.0
        self.marker.pose.position.z = 1.0
        # create a default line
        p0 = Point(x=-2.0,y=1.0,z=0.0)
        p1 = Point(x=-2.0,y=1.0,z=1.0)
        self.marker.points.append(p0)
        self.marker.points.append(p1)
        self.get_logger().info(f"added a new line_strip with namespace {namespc} ")


    # def publish_linelist(self,namespc):
    #     self.publisher_.publish(self.marker)


    def publish_marker(self):
        self.publisher_.publish(self.marker)

    def add_line(self,linept0,linept1):
        self.marker.points.append(linept0)
        self.marker.points.append(linept1)

    def add_point(self,pt2add):
        self.marker.points.append(pt2add)


    def set_ns(self,namespc):
        self.marker.ns = namespc

    def add_ns(self,namespc):
        self.marker.ns = namespc

class LineStrip(Node):
    def __init__(self):
        # define the node name
        super().__init__('rviz_marker_linestrip')
        self.publisher_ = self.create_publisher(Marker, 'line_strip_hey', 10)  

        self.marker = Marker()
        self.marker.header.frame_id = '/base_link'
        #self.marker.header.frame_id = '/map'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = self.marker.LINE_STRIP
        self.marker.id = 0
        self.marker.action = self.marker.ADD
        self.marker.ns = "salut" # 0.1 # only scale x is used for line strip markers, it's the width of the line
        self.marker.scale.x = 0.1 # only scale x is used for line strip markers, it's the width of the line
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 1.0
        self.marker.pose.position.z = 1.0
        p = Point()
        p.x = 2.0
        p.y = 0.0
        p.z = 0.0
        self.marker.points.append(p)
        self.get_logger().info("Publishing the line_strip topic. ")
    def publish_marker(self):
        self.publisher_.publish(self.marker)

    def add_point(self,pt2add):
        self.marker.points.append(pt2add)

class BalloonList(Node):
    def __init__(self):
        # define the node name
        super().__init__('rviz_marker_ballonlist')
        self.publisher_ = self.create_publisher(Marker, 'balloon_list', 10)  

        self.marker = Marker()
        self.marker.header.frame_id = '/base_link'
        #self.marker.header.frame_id = '/map'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = self.marker.SPHERE_LIST
        self.marker.id = 0
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 1.0
        self.marker.pose.position.z = 1.0
        p = Point()
        p.x = 1.0
        p.y = 1.0
        p.z = 1.0
        self.marker.points.append(p)
        self.get_logger().info("Publishing the balloon_list topic. Use RViz to visualize.")
    def publish_marker(self):
        self.publisher_.publish(self.marker)

    def add_point(self,pt2add):
        self.marker.points.append(pt2add)

class Balloon(Node):
    def __init__(self):
        super().__init__('rviz_visu_markers')
        #super().__init__('rviz_marker_tuto')
        self.publisher_ = self.create_publisher(Marker, 'balloon', 10)  

        self.marker = Marker()
        self.marker.header.frame_id = '/base_link'
        #self.marker.header.frame_id = '/map'
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.type = self.marker.SPHERE
        #self.marker.type = self.marker.SPHERE_LIST
        self.marker.id = 0
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.ns = 'ballon_namesp'
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 2.0
        self.get_logger().info("Publishing the balloon topic. Use RViz to visualize.")
    def publish_marker(self):
        self.publisher_.publish(self.marker)

def main(args=None):
    # atchoum
    rclpy.init(args=args)
    linelist2 = LineList2()
    # linelist2 = LineList2(namespc='ns2')
    myptstamped = MyPointStamped()
    arrow = Arrow('f','arrow1')
    arrow2 = Arrow('g','arrow2')
    # define by start and end points
    arrow2.start_end(Point(x=0.0,y=0.0,z=0.0),Point(x=0.0,y=1.0,z=1.0))
    # define by pose and orientation
    arrow.pos_orient(Point(x=0.0,y=0.0,z=0.0),2.0)
    balloon = Balloon()
    balloonlist = BalloonList()
    p0 = Point(x=0.1,y=0.1,z=0.1)
    p = Point()
    p.x = 2.0
    p.y = 1.0
    p.z = 1.0
    balloonlist.add_point(p)
    balloonlist.add_point(p0)
    balloonlist.add_point(Point(x=0.1,y=0.1,z=0.1))
    linestrip = LineStrip()
    p0 = Point(x=2.0,y=2.0,z=1.0)
    p = Point()
    p.x,p.y,p.z = 2.0,2.0,0.0
    linestrip.add_point(p)
    linestrip.add_point(p0)
    linestrip.add_point(Point(x=2.0,y=0.0,z=2.0))

    linelist = LineList(namespc='ns2')
    linelist.add_line(Point(x=2.0,y=0.5,z=0.0),Point(x=2.0,y=0.0,z=2.0))
    # linelist.add_ns('toto')
    # linelist.add_line(Point(x=2.0,y=0.0,z=0.0),Point(x=2.0,y=0.0,z=2.0))
    # linelist.add_linestrip('pet')
    # linelist.add_line(Point(x=2.0,y=1.0,z=0.0),Point(x=2.0,y=0.0,z=2.0))
    # myptstamped.publish_point(1.0,1.0,1.0)
    # rclpy.spin(myptstamped)
    # linelist2.publish_linelist_pt2('myns0',(2.0,1.0,1.0),(2.0,2.0,2.0))
    linelist2.publish_linelist_pt2('myns0',1,(2.0,1.0,3.0),(-2.0,2.0,2.0))
    linelist2.publish_linelist_pt2('myns0',2,(-2.0,1.0,3.0),(-2.0,2.0,2.0))
    while rclpy.ok():
        arrow.publish_marker()
        arrow2.publish_marker()
        balloon.publish_marker()
        balloonlist.publish_marker()
        linestrip.publish_marker()
        linelist.publish_marker()
        # linelist2.publish_linelist()
        linelist2.publish_linelist('myns')
        linelist2.publish_linelist('myns2')
        linelist2.publish_linelist_pt('myns3',(1.0,1.0,1.0),(2.0,2.0,2.0))
        linelist2.publish_marker()
        # myptstamped.publish_point()
    arrow.destroy_node()  
    arrow2.destroy_node()  
    myptstamped.destroy_node()  
    balloon.destroy_node()  
    balloonlist.destroy_node()  
    linestrip.destroy_node()  
    linelist.destroy_node()  
    linelist2.destroy_node()  
    rclpy.shutdown()
if __name__ == '__main__':
    main()
