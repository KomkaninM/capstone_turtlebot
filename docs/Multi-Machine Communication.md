# Multi-Machine Communication

Firstly, Turtlebot and PC must connect at same ip address \
Checking the ip

    ip -4 addr show | grep -oP 'inet \K[\d.]+/\d+'

Then run these on both turtlebot and your PC

    export ROS_DOMAIN_ID=30
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_LOCALHOST_ONLY=

