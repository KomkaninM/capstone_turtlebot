# TurtleBot Setup Guide

## Table of Contents
- [TurtleBot Setup Guide](#turtlebot-setup-guide)
  - [Table of Contents](#table-of-contents)
  - [2D Lidar Setup](#2d-lidar-setup)
  - [Multi-Machine Communication](#multi-machine-communication)
  - [Running Laser Lines Nodes for Visualization](#running-laser-lines-nodes-for-visualization)
  - [SSH Address List](#ssh-address-list)
  - [TurtleBot3 Drive Setup](#turtlebot3-drive-setup)

---

## 2D Lidar Setup

This step configures and verifies that your 2D LiDAR is connected and working properly.

1. **Check which port your LiDAR is connected to.**  
   It should appear as `/dev/ttyACM0` or `/dev/ttyACM1`.

    ```bash
    ls /dev/ttyACM* 
    sudo chmod 666 /dev/ttyACM0
    sudo chmod 666 /dev/ttyACM1
    ```

2. **Check that LiDAR-related ROS packages are installed:**

    ```bash
    ros2 pkg list | grep urg
    ```
    You should see something like:
    ```
    urg_c
    urg_node
    urg_node2
    urg_node_msgs
    ```

3. **Launch the LiDAR driver:**

    ```bash
    ros2 launch urg_node2 urg_node2.launch
    ```

4. **If you get an error**, try specifying the exact port:
    ```bash
    ros2 launch urg_node2 urg_node2.launch serial_port:="/dev/ttyACM1"
    ```

> ✅ **Check:** You should see laser scan data being published to `/scan`.
"
  
---

## Multi-Machine Communication

This allows your **PC** (for visualization) and **TurtleBot** (the robot) to communicate over the same network using ROS2.

1. **Make sure both devices are connected to the same Wi-Fi.**
2. **Check the IP address** of each device:

    ```bash
        ip -4 addr show | grep -oP 'inet \K[\d.]+/\d+'
    ```
    
3. **Set ROS environment variables** on both TurtleBot and PC:

    ```bash
        export ROS_DOMAIN_ID=30
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export ROS_LOCALHOST_ONLY=
    ```

---

## Running Laser Lines Nodes for Visualization
This visualizes LiDAR data as lines

1. **Run node that publishes only `/scan`:**

        ros2 run laser_lines scan_only

2. **Run node that publishes `/best_angle` and `/scan`:**

        ros2 run laser_lines best_angle_scan

---

## SSH Address List

Use these addresses to remotely connect to your TurtleBots via SSH.

**true_home5G_668**

    ssh ubuntu@172.20.10.7

**yawweir**

    ssh ubuntu@172.20.10.2


---


## TurtleBot3 Drive Setup

This launches the TurtleBot3 base and motor controllers.

📖 **References:**
- [Robotis TurtleBot3 Bringup Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup)
- [Robotis TurtleBot3 Teleoperation Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation)

---

1. **Set the robot model and launch:**

    ```bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_bringup robot.launch.py
    ```

2. **If you see errors or the motors don’t move, update OpenCR firmware:**

    ```bash
    cd ./opencr_update 
    export OPENCR_PORT=/dev/ttyACM0  # Check your port
    export OPENCR_MODEL=burger
    rm -rf ./opencr_update.tar.bz2 
    ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
    ```

> ✅ **Check:** When successful, you should hear a startup sound and see log messages like  
> `[turtlebot3_ros-3] [INFO] [diff_drive_controller]: Init Odometry`
> `[turtlebot3_ros-3] [INFO] [diff_drive_controller]: Run!`
