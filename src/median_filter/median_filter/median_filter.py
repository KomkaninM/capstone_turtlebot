import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import math
import numpy as np
from scipy.signal import savgol_filter, medfilt, find_peaks
from scipy.ndimage import median_filter

class WeldDetectorMedian(Node):

    def __init__(self):
        super().__init__('weld_detector_median')

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

        # --- PARAMETERS (Matching your MATLAB script) ---
        # Region of Interest (ROI) Indices
        self.roi_start = 330
        self.roi_end = 438  

        # Smoothing (Savitzky-Golay)
        self.sg_order = 3
        self.sg_framelen = 15 # Must be odd
        
        # Background Estimation (Median Filter)
        self.med_window = 51  # Must be odd
        
        # Peak Finding
        self.min_prominence = 0.001
        self.min_width = 2
        self.max_width = 50

        self.get_logger().info("Weld Detector (Median Filter Method) Started.")

    def scan_callback(self, msg):
        # 1. Slice the data to the specific indices
        # Note: Python slicing is [start:end_exclusive], so we add 1 to include 438
        current_ranges = np.array(msg.ranges[self.roi_start : self.roi_end + 1])

        # 2. Pre-processing: Handle NaNs and Infs
        # Replace 'inf' with max_range and 'nan' with 0.0 to prevent crashes
        current_ranges[np.isinf(current_ranges)] = msg.range_max
        current_ranges[np.isnan(current_ranges)] = 0.0

        # Safety check: Ensure we have enough data points for the filters
        if len(current_ranges) < self.med_window:
            self.get_logger().warn(f"ROI too small ({len(current_ranges)}) for filter window ({self.med_window})")
            return

        try:
            # --- STEP 1: SMOOTHING (Savitzky-Golay) ---
            smooth_signal = savgol_filter(current_ranges, self.sg_framelen, self.sg_order)

            # --- STEP 2: BACKGROUND ESTIMATION (Median Filter) ---
            background = median_filter(smooth_signal, size=self.med_window, mode='nearest')

            # --- STEP 3: SUBTRACTION ---
            # Invert the signal so the weld valley becomes a peak
            flattened_signal = background - smooth_signal

            # --- STEP 4: PEAK FINDING ---
            # Python: find_peaks returns indices and a dictionary of properties
            peaks, properties = find_peaks(
                flattened_signal, 
                prominence=self.min_prominence, 
                width=self.min_width
            )

            # --- STEP 5: SELECTION ---
            best_angle = float('nan')
            found_weld = False

            if len(peaks) > 0:
                prominences = properties['prominences']
                widths = properties['widths']

                # Sort peaks by prominence (Descending)
                # argsort gives ascending, so we flip with [::-1]
                sorted_indices = np.argsort(prominences)[::-1]

                for idx in sorted_indices:
                    width = widths[idx]
                    
                    # Validate max width constraint
                    if width <= self.max_width:
                        # We found the best valid peak!
                        local_peak_index = peaks[idx]
                        
                        # Convert Local Index -> Global Index -> Angle
                        global_index = self.roi_start + local_peak_index
                        
                        # Formula: angle = min + (index * inc)
                        best_angle = msg.angle_min + (global_index * msg.angle_increment)
                        found_weld = True
                        
                        # Debugging info
                        self.get_logger().info(f"Weld found at index {local_peak_index}, Prominence: {prominences[idx]:.4f}")
                        break 

            # --- STEP 6: PUBLISH ---
            msg_out = Float32()
            
            if found_weld:
                msg_out.data = float(best_angle)
            else:
                # Publish NaN if no weld found (standard practice to indicate lost tracking)
                msg_out.data = float('nan')
                # self.get_logger().info("No weld found.")

            self.angle_pub.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

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