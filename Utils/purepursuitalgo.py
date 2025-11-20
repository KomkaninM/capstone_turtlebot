# This node subscribes to /best_angle (a Float32)
# and publishes to /cmd_vel (a Twist)
# Uses Pure Pursuit algorithm for seam tracking

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

# --- Pure Pursuit Parameters ---
# Your lidar detects the seam 30cm ahead of the robot
# This detected position IS your lookahead point!
LOOKAHEAD_DISTANCE = 0.3  # meters (matches your seam detection distance)
LINEAR_VELOCITY = 0.2     # m/s (constant forward speed)
MAX_ANGULAR_VELOCITY = 1.0  # rad/s (maximum turning speed)

# Note: The lookahead distance should match where your sensor detects the seam
# If you want smoother tracking, you can increase this to 0.35-0.4m
# But don't go below 0.3m since that's where your actual detection is

# Smoothing filter parameter (0.0 = no filtering, 1.0 = maximum filtering)
ALPHA = 0.3  # Exponential moving average coefficient

class SeamTrackerPurePursuit(Node):

    def __init__(self):
        super().__init__('seam_tracker_pure_pursuit_node')
        
        # Create Subscriber for seam angle
        self.angle_subscriber = self.create_subscription(
            Float32,
            '/best_angle',
            self.pure_pursuit_callback,
            10)
            
        # Create Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        # State variables
        self.current_angle = 0.0
        self.filtered_angle = 0.0
        self.last_angular_vel = 0.0
        
        # Declare parameters (can be changed via command line or launch file)
        self.declare_parameter('lookahead_distance', LOOKAHEAD_DISTANCE)
        self.declare_parameter('linear_velocity', LINEAR_VELOCITY)
        self.declare_parameter('max_angular_velocity', MAX_ANGULAR_VELOCITY)
        self.declare_parameter('alpha', ALPHA)
        
        self.get_logger().info("Seam Tracker Pure Pursuit node is running...")
        self.get_logger().info(f"Lookahead: {LOOKAHEAD_DISTANCE}m, "
                              f"Linear vel: {LINEAR_VELOCITY}m/s")

    def pure_pursuit_callback(self, angle_msg):
        """
        Pure Pursuit algorithm implementation.
        
        Theory:
        - The seam angle represents the direction we want to follow
        - We calculate a lookahead point based on this angle
        - The curvature to reach that point determines our angular velocity
        """
        
        # Get parameters
        L = self.get_parameter('lookahead_distance').value
        v = self.get_parameter('linear_velocity').value
        max_omega = self.get_parameter('max_angular_velocity').value
        alpha = self.get_parameter('alpha').value
        
        # Get the current seam angle (in radians)
        self.current_angle = angle_msg.data
        
        # Apply exponential moving average filter for smoothing
        self.filtered_angle = (alpha * self.current_angle + 
                              (1 - alpha) * self.filtered_angle)
        
        # --- Pure Pursuit Calculation ---
        
        # Calculate the lookahead point in robot frame
        # The seam angle tells us the direction of the seam relative to robot
        lookahead_x = L * math.cos(self.filtered_angle)
        lookahead_y = L * math.sin(self.filtered_angle)
        
        # Calculate the curvature (inverse of turning radius)
        # Pure Pursuit formula: curvature = 2 * y / L^2
        # where y is the lateral offset of the lookahead point
        curvature = (2.0 * lookahead_y) / (L * L)
        
        # Calculate angular velocity
        # omega = v * curvature
        angular_velocity = v * curvature
        
        # Apply velocity limits
        angular_velocity = max(min(angular_velocity, max_omega), -max_omega)
        
        # Optional: Add smoothing to angular velocity changes
        # This prevents jerky movements
        smoothing_factor = 0.7
        angular_velocity = (smoothing_factor * angular_velocity + 
                          (1 - smoothing_factor) * self.last_angular_vel)
        self.last_angular_vel = angular_velocity
        
        # --- Publish Command ---
        twist_msg = Twist()
        twist_msg.linear.x = v  # Pure Pursuit moves forward while tracking
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = angular_velocity
        
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Optional: Log debug information
        if abs(self.filtered_angle) > 0.1:  # Only log significant angles
            self.get_logger().debug(
                f"Angle: {math.degrees(self.filtered_angle):.2f}°, "
                f"Curvature: {curvature:.3f}, "
                f"Angular vel: {angular_velocity:.3f} rad/s")


def main(args=None):
    rclpy.init(args=args)
    seam_tracker = SeamTrackerPurePursuit()
    
    try:
        rclpy.spin(seam_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutting down
        stop_msg = Twist()
        seam_tracker.cmd_vel_publisher.publish(stop_msg)
        seam_tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()