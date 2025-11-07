# 2D Lidar Setup

1. Setup Port of 2D Lidar (It should show as `/dev/ttyACM0`) 

        ls /dev/ttyACM* 
        sudo chmod 666 /dev/ttyACM0

2. Setup ROS2

        source /opt/ros/humble/setup.bash
        source ~/turtlebot3_ws/install/setup.bash   
        source ~/capstone_ws/install/setup.bash    

3. Checking Package list (It should show as `urg_c, urg_node, urg_node_msgs`)

        ros2 pkg list | grep urg

4. Launch LiDAR 

        ros2 run urg_node urg_node_driver --ros-args -p serial_port:=/dev/ttyACM0 -p frame_id:=laser
