# TurtleBot3 Motor Setup

Setup Robot  Launcher

        export TURTLEBOT3_MODEL=burger
        ros2 launch turtlebot3_bringup robot.launch.py

If  it launch successful, It should show like this
> [INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2025-10-21-16-31-30-154522-turtlebot-1451 \
> [INFO] [launch]: Default logging verbosity is set to INFO \
> urdf_file_name : turtlebot3_burger.urdf \
> [INFO] [robot_state_publisher-1]: process started with pid [1453] \
> [INFO] [hlds_laser_publisher-2]: process started with pid [1455] \
> [INFO] [turtlebot3_ros-3]: process started with pid [1457] \
> [hlds_laser_publisher-2] [INFO] [1761039091.219470704] [hlds_laser_publisher]: Init hlds_laser_publisher Node Main \
> [hlds_laser_publisher-2] [INFO] [1761039091.220177868] [hlds_laser_publisher]: port : /dev/ttyUSB0 frame_id : base_scan \
> [robot_state_publisher-1] [INFO] [1761039091.335737927] [robot_state_publisher]: got segment base_footprint \
> [robot_state_publisher-1] [INFO] [1761039091.336103017] [robot_state_publisher]: got segment base_link \
> [robot_state_publisher-1] [INFO] [1761039091.336164406] [robot_state_publisher]: got segment base_scan \
> [robot_state_publisher-1] [INFO] [1761039091.336203572] [robot_state_publisher]: got segment caster_back_link \
> [robot_state_publisher-1] [INFO] [1761039091.336238739] [robot_state_publisher]: got segment imu_link \
> [robot_state_publisher-1] [INFO] [1761039091.336275350] [robot_state_publisher]: got segment wheel_left_link \
> [robot_state_publisher-1] [INFO] [1761039091.336308368] [robot_state_publisher]: got segment wheel_right_link \
> [turtlebot3_ros-3] [INFO] [1761039091.350677221] [turtlebot3_node]: Init TurtleBot3 Node Main \
> [turtlebot3_ros-3] [INFO] [1761039091.353476447] [turtlebot3_node]: Init DynamixelSDKWrapper \
> [turtlebot3_ros-3] [INFO] [1761039091.360776558] [DynamixelSDKWrapper]: Succeeded to open the port(/dev/ttyACM0)! \
> [turtlebot3_ros-3] [INFO] [1761039091.370232380] [DynamixelSDKWrapper]: Succeeded to change the baudrate! \
> [ERROR] [hlds_laser_publisher-2]: process has died [pid 1455, exit code 255, cmd '/opt/ros/humble/lib/hls_lfcd_lds_driver/hlds_laser_publisher --ros-args -r __node:=hlds_laser_publisher -r __ns:=/ --params-file /tmp/launch_params_kfbtch_m'] \
> [turtlebot3_ros-3] [INFO] [1761039091.407413321] [turtlebot3_node]: Start Calibration of Gyro \
> [turtlebot3_ros-3] [INFO] [1761039096.407700951]_]()


If error,  Checking/Update OpenCR  (It should have sound when  finish!)

        cd ./opencr_update 
        export OPENCR_PORT=/dev/ttyACM0  #Check the port name
        export OPENCR_MODEL=burger
        rm -rf ./opencr_update.tar.bz2 
        ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr 
