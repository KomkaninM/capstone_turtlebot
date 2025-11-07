import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

import math
import numpy as np
np.set_printoptions(precision=2) # Print only 2 decimal points

from scipy.signal import savgol_filter
from scipy.signal import find_peaks
from scipy.signal import peak_prominences

def angles_to_index(angle_rad: float, angle_min: float, angle_increment: float) :
    return int((angle_rad - angle_min)//angle_increment)

def findClosestPeak(peakAngles, peakRanges, desired_value):
    
    min_distance = 6.28
    if len(peakAngles) == 0:
        location = math.nan
        idx = math.nan
        peakRange = math.nan
    else :
        for i, peakAngle in enumerate(peakAngles) :
            diff = abs(peakAngle - desired_value)
            if diff < min_distance :
                min_distance = diff
                location = peakAngle
                idx = i
                peakRange = peakRanges[i]
    return [location, idx, min_distance, peakRange]

class moving_average(Node):

    def __init__(self):
        super().__init__('closest_peak')

        self.scan_subs = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)

        self.best_angle_ = self.create_publisher(Float32, '/best_angle', 10)
        self.get_logger().info('Moving average node is running.')

        # Variables
        self.first_run = True
        self.reset_angle = 0

        # Interested Angle (Deg)
        self.interest_angle_min = math.radians(-30)
        self.interest_angle_max = math.radians(30)

        #Savitzky-Golay Filtering
        self.poly = 2
        self.win = 11
        self.best_angle = 0
        self.last_valid_angle = 0

        #Peaks Properties
        self.min_prominence = 0.005
        self.N_peak = 5
        self.min_angle_diff = math.radians(2)
        
        #Loop Count for analyze
        # self.loop_count = 0


    def scan_callback(self, scans): 
        # self.loop_count += 1
        angle_min = scans.angle_min
        angle_incr = scans.angle_increment
        # index_min = angles_to_index(self.interest_angle_min, angle_min, angle_incr)
        # index_max = angles_to_index(self.interest_angle_max, angle_min, angle_incr)
        index_min = 300
        index_max = 468

        interested_ranges_data = scans.ranges[index_min:index_max]
        valid_interested_ranges = []
        valid_interested_angles = []
        
        for local_index , ranges in enumerate(interested_ranges_data):
            valid_interested_angles.append(angle_min + (index_min + local_index) * angle_incr)
            if math.isfinite(ranges):
                valid_interested_ranges.append(ranges)
            elif math.isinf(ranges):
                valid_interested_ranges.append(scans.range_max)
            else :
                valid_interested_ranges.append(0.0)

        
        filter_ranges = savgol_filter(valid_interested_ranges, self.win, self.poly)
        np_angles = np.asarray(valid_interested_angles)
        # self.get_logger().info(f'{self.loop_count}')
        # print(round(valid_interested_ranges[0],4),round(valid_interested_ranges[50],4), round(valid_interested_ranges[51],4))
        # print(f'Loop : {self.loop_count} - First Data : {valid_interested_ranges[0]}')  

        peak_index, prop = find_peaks(-1 * filter_ranges, prominence=self.min_prominence)
        proms, _, _ = peak_prominences(-1 * filter_ranges, peak_index)

        # Have to experiment to check. which one is better

        # Finding candidates from all peaks (by prominence)
        sort_proms = np.argsort(proms)[::-1]
        keep_proms = sort_proms[:min(self.N_peak,len(sort_proms))]
        candidates_index = peak_index[keep_proms]
        top_angles_candidates = np_angles[candidates_index].tolist()
        top_ranges_candidates = filter_ranges[candidates_index].tolist()


        # Sort by height
        # peak_heights = (-1 * filter_ranges)[peak_index]
        # order_height = np.argsort(peak_heights)[::-1]
        # keep_height = order_height[:min(self.N_peak,len(order_height))]
        # candidates_index_height = peak_index[keep_height]
        # top_angles_candidates = np_angles[candidates_index_height].tolist()
        # top_ranges_candidates = filter_ranges[candidates_index_height].tolist()

        # top_angles_candidates_DEGREE = [math.degrees(angle) for angle in top_angles_candidates]


        if self.first_run :
            if not top_angles_candidates: # Check if the list is empty
                self.get_logger().warn("No peaks found.")
                self.reset_angle += 1
                return

            # Proms for First Peek
            sort_proms = np.argsort(proms)[::-1]
            keep_proms = sort_proms[:min(self.N_peak,len(sort_proms))]
            candidates_index = peak_index[keep_proms]
            top_angles_candidates = np_angles[candidates_index].tolist()
            top_ranges_candidates = filter_ranges[candidates_index].tolist()
            self.best_angle = top_angles_candidates[0]
            self.last_valid_angle = top_angles_candidates[0]
            self.first_run = False
        else :
            [location, idx, min_distance, peakRange] = findClosestPeak(top_angles_candidates, top_ranges_candidates, self.last_valid_angle)
            if min_distance <= self.min_angle_diff:
                self.best_angle = location
                self.last_valid_angle = location
                self.reset_angle = 0
            else :
                self.best_angle = math.nan 
                self.reset_angle += 1
        angle_package = Float32()
        angle_package.data = float(self.best_angle)
        self.best_angle_.publish(angle_package)

        best_angle_output = f"Best Angle: {round(math.degrees(self.best_angle),4)} deg"
        valid_angle_output = f"Last Valid Angle: {round(math.degrees(self.last_valid_angle),4)}"
        log_message = f"{best_angle_output:<35} | {valid_angle_output}"
        self.get_logger().info(log_message)

        if self.reset_angle == 5:
            self.first_run = True # Reset like it is first run
            self.reset_angle = 0
       
def main(args=None):
    rclpy.init(args=args)

    detection_node = moving_average()

    rclpy.spin(detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()