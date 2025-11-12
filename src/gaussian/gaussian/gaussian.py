import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import math
import numpy as np
from scipy.signal import savgol_filter
from scipy.signal import find_peaks
from scipy.signal import peak_prominences

def angles_to_index(angle_rad: float, angle_min: float, angle_increment: float) :
    return int((angle_rad - angle_min)//angle_increment)

def findClosestPeak(peak_angles_np: np.ndarray, desired_value: float):
    if peak_angles_np.size == 0:
        return math.nan, math.nan, 6.28 # Return large distance

    # Calculate all differences at once
    diffs = np.abs(peak_angles_np - desired_value)
    
    # Find the index *within the peak_angles_np array*
    idx_in_list = np.argmin(diffs)
    min_distance = diffs[idx_in_list]
    location = peak_angles_np[idx_in_list]
    
    return [location, idx_in_list, min_distance]

class gaussian(Node):

    def __init__(self):
        super().__init__('gaussian_method')

        self.scan_subs = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)

        self.best_angle_ = self.create_publisher(Float32, '/best_angle', 10)
        self.get_logger().info('Gaussian node is running.')

        # Variables
        self.first_run = True
        self.first_setup = True
        self.reset_angle = 0

        # Interested Angle (Deg)
        self.interest_angle_min = math.radians(-30)
        self.interest_angle_max = math.radians(30)

        # Savitzky-Golay Filtering
        self.poly = 2
        self.win = 11
        self.best_angle = 0
        self.last_valid_angle = 0
        
        # Gaussian Parameters
        self.mu_idx = 50      # Start the "magnet" at Middle Index
        self.sigma = 5          # Width of magnet in INDICES
        self.gaussian_divider = 5

        # Find Peaks
        self.min_prominence = 0.001
        self.N_peak = 5
        self.min_angle_diff = math.radians(3)

        # Loop Count
        # self.loop_count = 0

    def scan_callback(self, scans): 
        angle_min = scans.angle_min
        angle_incr = scans.angle_increment
        index_min = 300
        index_max = 468

        # --- 1. Fast NumPy Data Filtering ---
        indices = np.arange(index_min, index_max)
        np_angles = angle_min + indices * angle_incr
        np_ranges = np.array(scans.ranges[index_min:index_max])

        # Replace all invalid values at once
        np_ranges[np.isinf(np_ranges)] = scans.range_max
        np_ranges[np.isnan(np_ranges)] = 0.0
        
        # --- 2. Simplified First Run Setup ---
        if self.first_run:
            self.data_point = len(np_angles)
            self.mu_idx = self.data_point // 2  # Start magnet in the middle
            self.x_indices = np.arange(self.data_point) # e.g., [0, 1, ... 167]

        # --- 3. Gaussian Magnet ---
        term1 = 1 / (self.sigma * np.sqrt(2 * np.pi))
        term2 = np.exp(-((self.x_indices - self.mu_idx)**2) / (2 * self.sigma**2))
        gaussian_magnet = (term1 * term2) / self.gaussian_divider

        # Apply filter and magnet
        filter_ranges = savgol_filter(np_ranges, self.win, self.poly)
        transform_ranges = filter_ranges - gaussian_magnet

        # --- 4. Find Peaks (Prominence Bug Fixed) ---
        peak_index, prop = find_peaks(-1 * transform_ranges, prominence=self.min_prominence)
        
        # FIX: Get prominences from 'prop' dictionary
        proms = prop['prominences'] 

        if peak_index.size == 0:
            self.get_logger().warn("No peaks found.")
            self.best_angle = math.nan
            self.reset_angle += 1
            # Still publish the 'nan' angle
        else:
            # --- 5. Sort Peaks and Get Candidates ---
            sort_proms_indices = np.argsort(proms)[::-1]
            keep_proms_indices = sort_proms_indices[:min(self.N_peak, len(sort_proms_indices))]
            
            # These are the indices relative to the slice (e.g., 0-167)
            candidates_index = peak_index[keep_proms_indices]
            
            top_angles_candidates = np_angles[candidates_index]

            # --- 6. Tracking Logic (First Run Fixed) ---
            if self.first_run:
                # Lock onto the most prominent peak
                self.best_angle = top_angles_candidates[0]
                self.last_valid_angle = self.best_angle
                
                # FIX: Update the magnet's position
                self.mu_idx = candidates_index[0] 
                
                self.first_run = False
                self.reset_angle = 0

            else:
                # Use the fast peak finder
                [location, idx_in_list, min_distance] = findClosestPeak(
                    top_angles_candidates, self.last_valid_angle
                )
                
                if min_distance <= self.min_angle_diff:
                    self.best_angle = location
                    self.last_valid_angle = location
                    self.reset_angle = 0
                    
                    # FIX: Update the magnet's position to the new peak's index
                    self.mu_idx = candidates_index[idx_in_list]
                else:
                    self.best_angle = math.nan 
                    self.reset_angle += 1
                    # Don't update mu_idx, let the magnet stay where it was

        # --- 7. Publish and Log (Logging Bug Fixed) ---
        angle_package = Float32()
        angle_package.data = float(self.best_angle)
        self.best_angle_.publish(angle_package)

        if math.isnan(self.best_angle):
            best_angle_output = "Best Angle: nan (Tracking Lost)"
        else:
            best_angle_output = f"Best Angle: {round(math.degrees(self.best_angle),4)} deg"
        
        if math.isnan(self.last_valid_angle):
            valid_angle_output = "Last Valid Angle: nan"
        else:
            valid_angle_output = f"Last Valid Angle: {round(math.degrees(self.last_valid_angle),4)}"
        
        log_message = f"{best_angle_output:<35} | {valid_angle_output}"
        self.get_logger().info(log_message)

        if self.reset_angle == 5:
            self.get_logger().warn("Lost track for 5 scans, resetting...")
            self.first_run = True # Reset to re-acquire
            self.reset_angle = 0

def main(args=None):
    rclpy.init(args=args)

    detection_node = gaussian()

    rclpy.spin(detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()