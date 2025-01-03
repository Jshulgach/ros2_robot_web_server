# ROS2 Robot Web Server
ROS2 wrappers for the Robot Web Server


## Installation
1. Clone the repository in your ROS2 workspace:
   ```bash
   cd /path/to/your/ros2/workspace/src
   git clone https://github.com/Jshulgach/ros2_robot_web_server.git
    ```
2. Build the package and source the workspace:
    ```bash
    cd ..
    colcon build --packages-select ros2_robot_web_server
    source install/setup.bash
    ```
   
## Usage
1. Run the Robot Web Server in a terminal. Specify the port and baudrate as arguments, or leave as-is for default values:
    ```bash
    ros2 run ros2_robot_web_server robot_server.py --port '/dev/ttyACM0' --baudrate 9600
    ```
   

   