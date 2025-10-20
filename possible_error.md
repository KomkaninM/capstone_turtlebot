# Possible Error Lists


1. The serial port /dev/ttyACM0 is already in use by another process, preventing the Hokuyo driver from accessing it. 

> [INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2025-10-17-13-51-45-308579-turtlebot-1564 \
    [INFO] [launch]: Default logging verbosity is set to INFO \
    [INFO] [urg_node_driver-1]: process started with pid [1566] \
    [urg_node_driver-1] [ERROR] [1760683905.923393212] [urg_node]: Error connecting to Hokuyo: Could not open serial Hokuyo: 


    sudo fuser /dev/ttyACM0
    sudo kill -9 {port_number}

