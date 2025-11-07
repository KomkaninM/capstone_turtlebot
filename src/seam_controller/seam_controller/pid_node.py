# This node subscribes to /seam_angle (a Float32)
# and publishes to /cmd_vel (a Twist)

import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32  # For subscribing to the angle
from geometry_msgs.msg import Twist   # For publishing motor commands

# --- PID Gains (You must tune these!) ---
KP = 0.7  # Proportional gain (Adjust this one first)
KI = 0.0  # Integral gain (Use to fix small, steady errors)
KD = 0.0  # Derivative gain (Use to prevent overshooting)

# Optional: Set a maximum turn speed
MAX_TURN_SPEED = 0.2  # Radians per second

class SeamTrackerPID(Node):

    def __init__(self):
        super().__init__('seam_tracker_node')
        
        # 1. Create Subscriber (listens for the angle)
        self.angle_subscriber = self.create_subscription(
            Float32,
            '/best_angle',
            self.pid_callback,  # Function to run every time an angle arrives
            10)
            
        # 2. Create Publisher (sends commands to motors)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
            
        # 3. Initialize PID variables
        self.previous_error = 0.0
        self.integral = 0.0
        
        self.get_logger().info("Seam Tracker PID node is running...")

    def pid_callback(self, angle_msg):
        """
        This is the PID calculation.
        It runs every time a new seam angle is received.
        """
        # --- Task 1: Get the Error ---
        # The 'error' is the angle itself.
        # Our goal (setpoint) is 0.
        error = angle_msg.data
        if math.isnan(error) or math.isinf(error):
            error = self.previous_error
        # --- Task 2: Calculate PID Correction ---
        
        # Proportional term (present error)
        P = KP * error
        
        # Integral term (accumulated past error)
        self.integral += error
        # Anti-windup: clamp the integral
        self.integral = max(min(self.integral, 1.0), -1.0) 
        I = KI * self.integral
        
        # Derivative term (rate of change)
        derivative = error - self.previous_error
        D = KD * derivative
        
        # --- Total correction is the turning speed ---
        # We use a negative sign to correct the error.
        # - If error is POSITIVE (left), we need a POSITIVE (left) turn.
        # (You may need to flip this sign: 'angular_speed = -total_correction'
        #  depending on your robot's coordinate frame)
        
        total_correction = P + I + D
        angular_speed = -total_correction
        
        # Save error for next loop
        self.previous_error = error

        # --- Task 3: Publish the Motor Command ---
        
        # Clamp the output to your maximum speed
        angular_speed = max(min(angular_speed, MAX_TURN_SPEED), -MAX_TURN_SPEED)

        # Create the Twist message
        twist_msg = Twist()
        twist_msg.linear.x = 0.0  ###
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = angular_speed # Set the turn speed
        print(f"The Current Angular Speed is {angular_speed}")
        # Publish the command
        self.cmd_vel_publisher.publish(twist_msg)

# (Boilerplate code to start the node)
def main(args=None):
    rclpy.init(args=args)
    seam_tracker = SeamTrackerPID()
    rclpy.spin(seam_tracker)
    seam_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()