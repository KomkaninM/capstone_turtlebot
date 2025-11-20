import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import numpy as np
import math
from scipy.signal import savgol_filter, find_peaks
from scipy.ndimage import median_filter

class WeldDetectorRobust(Node):

    def __init__(self):
        super().__init__('weld_detector_robust')

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

        # --- PARAMETERS (From MATLAB) ---
        # Region of Interest (Indices: 330 to 438 based on previous context)
        self.roi_start = 330
        self.roi_end = 438  

        # Signal Processing
        self.sg_order = 3
        self.sg_framelen = 15 
        self.med_window = 51  
        
        # Peak Finding thresholds
        self.min_prominence = 0.001
        self.max_width = 100
        self.min_height_threshold = 0.0 # Updated to 0 as per your script

        # --- TRACKING VARIABLES ---
        # We use NaN to represent "No previous valid angle" (like last_idx = -1)
        self.last_valid_angle = float('nan') 
        self.missed_count = 0
        self.reset_threshold = 5
        
        # Validation: 5 degrees in radians (Approx equivalent to 10 indices)
        self.angle_diff_threshold = math.radians(5) 

        self.get_logger().info("Weld Detector (Robust) Started.")

    # --- HELPER FUNCTIONS ---

    def index_to_angle(self, index, angle_min, angle_increment):
        """Converts a global scan index to an angle (radians)."""
        return angle_min + (index * angle_increment)

    def angle_to_index(self, angle, angle_min, angle_increment):
        """Converts an angle (radians) to a global scan index."""
        return int((angle - angle_min) / angle_increment)

    def check_validity(self, current_angle, past_angle):
        """
        Checks if the current angle is valid compared to the past angle.
        Matches MATLAB: if abs(curr - past) < 10 (indices) -> ~5 degrees
        """
        # If past_angle is NaN (reset or first run), ANY angle is valid
        if math.isnan(past_angle):
            return True
            
        diff = abs(current_angle - past_angle)
        return diff < self.angle_diff_threshold

    def scan_callback(self, msg):
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
            # Note: Scipy args are (x, window, polyorder)
            smooth_signal = savgol_filter(current_ranges, self.sg_framelen, self.sg_order)

            # 2. Background Estimation (Median)
            # mode='nearest' avoids zero-padding errors at edges
            background = median_filter(smooth_signal, size=self.med_window, mode='nearest')

            # 3. Flattening
            flattened_signal = background - smooth_signal

            # 4. Peak Finding
            # We request 'width' calculation here
            peaks, properties = find_peaks(
                flattened_signal, 
                prominence=self.min_prominence,
                width=0 
            )

            # --- CANDIDATE SELECTION ---
            found_weld = False
            best_angle = float('nan')
            
            # Variables for logging (Debug info for failed scans)
            debug_loc_idx = -1
            debug_height = 0.0

            if len(peaks) > 0:
                prominences = properties['prominences']
                widths = properties['widths']
                peak_heights = flattened_signal[peaks] # Actual height values

                # 1. Sort by prominence (Descending)
                sorted_indices = np.argsort(prominences)[::-1]

                # 2. Determine candidates (Max 3)
                num_candidates = min(len(sorted_indices), 3)

                # Save info about Candidate #1 just for logging purposes
                top_ptr = sorted_indices[0]
                debug_loc_idx = peaks[top_ptr] + self.roi_start
                debug_height = peak_heights[top_ptr]

                # 3. Loop through top candidates
                for k in range(num_candidates):
                    idx = sorted_indices[k]
                    
                    local_idx = peaks[idx]
                    current_width = widths[idx]
                    current_height = peak_heights[idx]
                    current_prom = prominences[idx]

                    # Convert Local Index -> Global Index -> Angle
                    global_idx = self.roi_start + local_idx
                    current_angle = self.index_to_angle(global_idx, msg.angle_min, msg.angle_increment)

                    # --- 4. VALIDATION LOGIC ---
                    
                    # A. Check Distance (Tracking)
                    loc_valid = self.check_validity(current_angle, self.last_valid_angle)

                    # B. Check Height
                    height_valid = (current_height >= self.min_height_threshold)

                    # 5. Final Decision
                    if (current_width <= self.max_width) and loc_valid and height_valid:
                        # SUCCESS! We found a valid peak.
                        found_weld = True
                        best_angle = current_angle
                        
                        # Update State
                        self.last_valid_angle = best_angle
                        self.missed_count = 0
                        
                        # Update Debug vars to show the winner
                        debug_loc_idx = global_idx
                        debug_height = current_height
                        
                        # Break loop (stop looking once we find a valid one)
                        break 

            # --- OUTPUT & RESET LOGIC ---
            if not found_weld:
                self.missed_count += 1
                
                # Log failure details
                self.get_logger().info(
                    f"No valid weld. Last Checked Idx: {debug_loc_idx}, H: {debug_height:.4f} "
                    f"(Miss: {self.missed_count}/{self.reset_threshold})"
                )

                if self.missed_count >= self.reset_threshold:
                    self.get_logger().warn("*** Resetting Tracking State ***")
                    self.last_valid_angle = float('nan') # Reset memory (last_idx = -1)
                    self.missed_count = 0
                    
                # Publish NaN
                msg_out = Float32()
                msg_out.data = float('nan')
                self.angle_pub.publish(msg_out)

            else:
                # Log success
                # self.get_logger().info(f"Weld Found at Idx {debug_loc_idx}. Height: {debug_height:.4f}")
                
                # Publish Best Angle
                msg_out = Float32()
                msg_out.data = float(best_angle)
                self.angle_pub.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"Processing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WeldDetectorRobust()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()