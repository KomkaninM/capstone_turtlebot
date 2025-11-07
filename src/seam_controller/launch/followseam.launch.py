from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seam_controller',
            executable='pid_tracker', # This is the 'entry_point' name!
            name='seam_tracker_node'  # This renames the node at runtime
        ),
        
        # --- Add your other nodes here too! ---
        # For example, your lidar processor:
        # Node(
        #     package='lidar_processor_pkg',
        #     executable='lidar_node',
        #     name='lidar_processor'
        # ),
        
        # And your motor driver:
        # Node(
        #     package='motor_driver_pkg',
        #     executable='driver_node',
        #     name='motor_driver'
        # )
    ])