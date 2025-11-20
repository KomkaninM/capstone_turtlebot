#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import numpy as np
import math
from scipy.signal import savgol_filter, find_peaks
from scipy.ndimage import median_filter

class WeldDetectorMedian(Node):

    def __init__(self):
        super().__init__('weld_detector_median')

        # --- SUBSCRIPTIONS & PUBLISHERS ---
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.angle_pub = self.create_publisher(
            Float32, 
            '/best_angle', 
            10
        )

        # --- PARAMETERS ---
        # Region of Interest (Indices)
        self.roi_start = 330
        self.roi_end = 438  

        # Signal Processing
        self.sg_order = 3
        self.sg_framelen = 15 
        self.med_window = 51  
        
        # Peak Finding
        self.min_prominence = 0.001
        self.max_width = 100
        self.min_height_threshold = 0.0075 

        # --- TRACKING VARIABLES ---
        # We use NaN to represent "No previous valid angle" (like last_idx = -1)
        self.last_valid_angle = float('nan') 
        self.missed_count = 0
        self.reset_threshold = 5
        
        # Validation: 5 degrees in radians
        self.angle_diff_threshold = math.radians(5) 

        # Check for /scan
        # Store the current time as the "last scan time"
        self.last_scan_time = self.get_clock().now()
        
        # How long to wait before resetting (3 seconds)
        self.scan_timeout_sec = 3.0 
        
        # Create a timer that runs every 0.5 seconds to check for timeout
        self.timer = self.create_timer(0.5, self.check_scan_timeout)
        self.timeout_triggered = False
        self.get_logger().info("Weld Detector (Median) Started.")

    def check_scan_timeout(self):
        current_time = self.get_clock().now()
        
        # Calculate time difference in seconds
        # (nanoseconds / 1e9 = seconds)
        time_diff = (current_time - self.last_scan_time).nanoseconds / 1e9
        
        if time_diff > self.scan_timeout_sec:
            # Only log/reset if we aren't already in a timeout state
            if not self.timeout_triggered:
                self.get_logger().warn(f"No Scan Data for {time_diff:.1f}s! Resetting State.")
                
                # --- RESET LOGIC ---
                self.last_valid_angle = float('nan')
                self.missed_count = 0
                self.timeout_triggered = True
                
                # Optional: Publish NaN to tell downstream nodes we are lost
                msg_out = Float32()
                msg_out.data = float('nan')
                self.angle_pub.publish(msg_out)

    # --- HELPER FUNCTIONS ---

    def index_to_angle(self, index, angle_min, angle_increment):
        """Converts a global scan index to an angle (radians)."""
        return angle_min + (index * angle_increment)

    def angle_to_index(self, angle, angle_min, angle_increment):
        """Converts an angle (radians) to a global scan index."""
        return int((angle - angle_min) / angle_increment)

    def is_valid_weld(self, current_angle, past_angle):
        """
        Checks if the current angle is within 5 degrees of the past angle.
        Matches MATLAB: if abs(curr - past) < 5
        """
        # If past_angle is NaN (reset or first run), ANY angle is valid
        if math.isnan(past_angle):
            return True
            
        diff = abs(current_angle - past_angle)
        return diff < self.angle_diff_threshold

    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        self.timeout_triggered = False
        
        # 1. Slice Data (ROI)
        # We get the subset of ranges we care about
        current_ranges = np.array(msg.ranges[self.roi_start : self.roi_end + 1])

        # 2. Basic Validation (Clean Data)
        current_ranges[np.isinf(current_ranges)] = msg.range_max
        current_ranges[np.isnan(current_ranges)] = 0.0

        # Safety check: need enough data for the median filter
        if len(current_ranges) < 50:
            return

        try:
            # --- PROCESSING ---
            
            # 1. Smoothing (Savitzky-Golay)
            smooth_signal = savgol_filter(current_ranges, self.sg_framelen, self.sg_order)

            # 2. Background Estimation (Median)
            # mode='nearest' matches MATLAB's 'truncate' behavior to avoid zero-padding edges
            background = median_filter(smooth_signal, size=self.med_window, mode='nearest')

            # 3. Flattening
            flattened_signal = background - smooth_signal

            # 4. Peak Finding
            # We request 'width' calculation here to match MATLAB logic
            peaks, properties = find_peaks(
                flattened_signal, 
                prominence=self.min_prominence,
                width=0 
            )

            # --- CANDIDATE SELECTION ---
            found_weld = False
            best_angle = float('nan')
            
            # Variables for logging the first candidate (if it fails)
            first_cand_idx = -1
            first_cand_height = 0.0

            if len(peaks) > 0:
                prominences = properties['prominences']
                widths = properties['widths']
                peak_heights = flattened_signal[peaks] # Actual height of the peak

                # Sort by prominence (Descending)
                sorted_indices = np.argsort(prominences)[::-1]

                # Check top 3 candidates
                num_candidates = min(len(sorted_indices), 3)

                # Save info about #1 for logging
                first_idx_ptr = sorted_indices[0]
                first_cand_idx = peaks[first_idx_ptr] + self.roi_start
                first_cand_height = peak_heights[first_idx_ptr]

                for k in range(num_candidates):
                    idx = sorted_indices[k]
                    
                    local_idx = peaks[idx]
                    current_width = widths[idx]
                    current_height = peak_heights[idx]

                    # Convert Local Index -> Global Index -> Angle
                    global_idx = self.roi_start + local_idx
                    current_angle = self.index_to_angle(global_idx, msg.angle_min, msg.angle_increment)

                    # --- VALIDATION LOGIC ---
                    
                    # A. Check Distance (Tracking)
                    # "is_valid_weld" checks if it's within 5 degrees of last valid
                    loc_valid = self.is_valid_weld(current_angle, self.last_valid_angle)

                    # B. Check Height (Optional based on your MATLAB code)
                    # height_valid = (current_height >= self.min_height_threshold)
                    height_valid = True 

                    # Final Decision
                    if (current_width <= self.max_width) and loc_valid and height_valid:
                        # SUCCESS! We found a valid peak.
                        found_weld = True
                        best_angle = current_angle
                        
                        # Update State
                        self.last_valid_angle = best_angle
                        self.missed_count = 0
                        
                        # Break loop (we take the best valid one)
                        break 

            # --- OUTPUT & RESET LOGIC ---
            if not found_weld:
                self.missed_count += 1
                
                # Log failure details (similar to MATLAB fprintf)
                self.get_logger().info(
                    f"No valid weld. Top Cand Idx: {peaks[first_idx_ptr]}, H: {first_cand_height:.4f} "
                    f"(Miss: {self.missed_count}/{self.reset_threshold})"
                )

                if self.missed_count >= self.reset_threshold:
                    self.get_logger().warn("*** Resetting Tracking State ***")
                    self.last_valid_angle = float('nan') # Reset memory
                    self.missed_count = 0
                    
                # Publish NaN
                msg_out = Float32()
                msg_out.data = float('nan')
                self.angle_pub.publish(msg_out)

            else:
                # Log success
                self.get_logger().info(f"Weld Found. Angle: {math.degrees(best_angle):.2f} deg, index: {peaks[first_idx_ptr]}")
                
                # Publish Best Angle
                msg_out = Float32()
                msg_out.data = float(best_angle)
                self.angle_pub.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"Processing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WeldDetectorMedian()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()