#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math


class ScanToLinesNode(Node):
    def __init__(self):
        super().__init__('scan_to_lines_node')
        
        # Subscriber to /scan topic
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for line markers
        self.marker_publisher = self.create_publisher(
            Marker,
            '/scan_lines',
            10
        )
        
        self.get_logger().info('LaserScan to Lines Node started!')
        self.get_logger().info('Subscribing to: /scan')
        self.get_logger().info('Publishing to: /scan_lines')
    
    def scan_callback(self, msg):
        # Create a LINE_LIST marker
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id  # Should be 'laser'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "laser_scan_lines"
        marker.id = 0
        marker.type = Marker.LINE_LIST  # LINE_LIST type for individual lines
        marker.action = Marker.ADD
        
        # Line properties
        marker.scale.x = 0.0005  # Line width (meters)
        
        # Green color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (transparency)
        
        # Lifetime (0 means forever, until next update)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        # Create line segments
        angle = msg.angle_min
        
        for i, range_value in enumerate(msg.ranges):
            # Skip invalid readings (NaN, inf, or out of range)
            if (math.isnan(range_value) or 
                math.isinf(range_value) or 
                range_value < msg.range_min or 
                range_value > msg.range_max):
                angle += msg.angle_increment
                continue
            
            # Set ranges of input
            if i < 330  or i > 420 :
                continue
                
       		# Calculate the endpoint of the line in polar coordinates
            # Convert to Cartesian coordinates
            x = range_value * math.cos(angle)
            y = range_value * math.sin(angle)
            
            # Start point (center/origin)
            start_point = Point()
            start_point.x = 0.0
            start_point.y = 0.0
            start_point.z = 0.0
            
            # End point (scan point)
            end_point = Point()
            end_point.x = x
            end_point.y = y
            end_point.z = 0.0
            
            # Add both points to create a line segment
            # LINE_LIST requires pairs of points
            marker.points.append(start_point)
            marker.points.append(end_point)
            
            # Move to next angle
            angle += msg.angle_increment
        
        # Publish the marker
        self.marker_publisher.publish(marker)
        
        # Log info occasionally (every 50 messages to avoid spam)
        if not hasattr(self, 'msg_count'):
            self.msg_count = 0
        
        self.msg_count += 1
        if self.msg_count % 50 == 0:
            self.get_logger().info(f'Published {len(marker.points)//2} lines')


def main(args=None):
    rclpy.init(args=args)
    node = ScanToLinesNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
