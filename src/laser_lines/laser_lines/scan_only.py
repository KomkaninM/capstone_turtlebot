import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

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
            qos_profile_sensor_data
        )

        # Publisher for line markers
        self.marker_publisher = self.create_publisher(
            Marker,
            '/scan_lines',
            10
        )

        self.laser_frame_id = 'laser'
        self.min_focus_index = 330
        self.max_focus_index = 438

        self.get_logger().info('LaserScan to Lines Node started!')
        self.get_logger().info('Subscribing to: /scan')
        self.get_logger().info('Publishing to: /scan_lines')
    
    def scan_callback(self, msg):
        # Create a LINE_LIST marker
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id  # Should be 'laser' or similar
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "laser_scan_lines"
        marker.id = 0
        marker.type = Marker.LINE_LIST  # LINE_LIST type for individual lines
        marker.action = Marker.ADD
        
        # Line properties (thickness)
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
        for i, range_value in enumerate(msg.ranges):

            if i < self.min_focus_index or i > self.max_focus_index:
                continue  # Skip this index

            if (math.isnan(range_value) or 
                math.isinf(range_value) or 
                range_value < msg.range_min or 
                range_value > msg.range_max):
                continue  # Skip this invalid range

            # Transform Coordination
            current_angle = msg.angle_min + (i * msg.angle_increment)
            x = range_value * math.cos(current_angle)
            y = range_value * math.sin(current_angle)
                       
            # Start point (center/origin of the laser)
            start_point = Point()
            start_point.x = 0.0
            start_point.y = 0.0
            start_point.z = 0.0
            
            # End point (the scan point)
            end_point = Point()
            end_point.x = x
            end_point.y = y
            end_point.z = 0.0
            
            marker.points.append(start_point)
            marker.points.append(end_point)
        
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
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
