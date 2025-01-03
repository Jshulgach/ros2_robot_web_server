# Python libraries
import os
import time
import datetime
import numpy as np

# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from ament_index_python.packages import get_package_share_directory


class PoseLoggerNode(Node):
    def __init__(self, verbose=False):
        super().__init__('pose_logger_node')
        self.verbose = verbose
        self.trajectory_index = 0
        self.latest_pose = None

        # Set up ROS2 parameters
        self.initialize_parameters()

        # Set up ROS2 publishers and subscribers
        self.pose_sub = self.create_subscription(PoseArray, self.pose_topic, self.pose_cb, 10)
        self.serial_pub = self.create_publisher(String, self.message_topic, 10)

        # Timer for sending robot arm commands and logging
        self.command_timer = self.create_timer(1/self.command_frequency, self.send_robot_command)
        self.logging_timer = self.create_timer(1/self.logging_frequency, self.log_pose_data)

        # Create log files
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.command_log_file = os.path.join(self.log_directory, f"command_log_{timestamp}.csv")
        self.pose_log_file = os.path.join(self.log_directory, f"pose_log_{timestamp}.csv")
        self.get_logger().info(f"Command log file: {self.command_log_file}")
        self.get_logger().info(f"Pose log file: {self.pose_log_file}")

        # Create the log file and write the header
        self.write_to_log(self.pose_log_file, "time,x,y,z\n", new_file=True)
        self.write_to_log(self.command_log_file, "time,x,y,z\n", new_file=True)

        # variables for trajectory generation
        self.trajectory = self.create_circular_trajectory(
            center=[0.135, 0.0, 0.25],
            radius=0.02,
            steps=101,
        )
        self.start_time = time.time()
        self.get_logger().info("PoseLoggerNode has started.")

    def initialize_parameters(self):
        """Initializes parameters from the parameter server."""
        self.declare_parameter('pose_topic', '/aruco/poses')
        self.declare_parameter("marker_id", 15)
        self.declare_parameter('message_topic', '/mini_arm/input/serial_message')
        self.declare_parameter('command_frequency', 0.1)
        self.declare_parameter('logging_frequency', 0.1)
        self.declare_parameter('log_file', 'pose_log.csv')
        self.declare_parameter('log_directory', '')

        self.pose_topic = self.get_parameter('pose_topic').value
        self.message_topic = self.get_parameter('message_topic').value
        self.command_frequency = self.get_parameter('command_frequency').value
        self.logging_frequency = self.get_parameter('logging_frequency').value
        self.log_directory = self.get_parameter('log_directory').value

        # Ensure the log directory exists
        if not self.log_directory:
            self.log_directory = self.get_log_directory()

        self.get_logger().info(f"Pose topic: {self.pose_topic}")
        self.get_logger().info(f"Message topic: {self.message_topic}")
        self.get_logger().info(f"Command frequency: {self.command_frequency}")
        self.get_logger().info(f"Logging frequency: {self.logging_frequency}")

    def get_log_directory(self):
        """Gets the 'log' directory within the package."""
        package_dir = get_package_share_directory('ros2_robot_web_server')
        log_directory = os.path.join(package_dir, 'log')
        os.makedirs(log_directory, exist_ok=True)
        return log_directory
    def write_to_log(self, filename, data, new_file=False):
        # Helper function to write data to the passed filename
        mode = 'w' if new_file else 'a'
        with open(filename, mode) as f:
            f.write(data)

    def create_circular_trajectory(self, center, radius=10, steps=101):
        """Creates a set of points along a circle in the Y-Z plane."""
        theta = np.linspace(0, 2 * np.pi, steps)
        temp = np.array([np.zeros(len(theta)), np.cos(theta), np.sin(theta)]).transpose()
        return center + radius * temp

    def pose_cb(self, msg):
        """Callback function to handle incoming pose data for the pecified marker id."""
        if msg.poses:
            #self.get_logger().info("Pose data received")
            self.latest_pose = msg.poses[-1]
            #self.get_logger().info(f"Pose updated: x={self.latest_pose.position.x}, y={self.latest_pose.position.y}, z={self.latest_pose.position.z}")
        else:
            self.get_logger().warn("No pose data received")

    def log_pose_data(self):
        """Logs the latest pose data to a file."""
        if self.trajectory_index < len(self.trajectory) and self.latest_pose:
            timestamp = time.time() - self.start_time
            x = self.latest_pose.position.x
            y = self.latest_pose.position.y
            z = self.latest_pose.position.z
            msg = f'{timestamp:.4f},{x:.4f},{y:.4f},{z:.4f}\n'
            self.write_to_log(self.pose_log_file, msg)
            self.get_logger().info(f"Logged pose: time={timestamp:.4f}, x={x:.4f}, y={y:.4f}, z={z:.4f}")
        else:
            self.get_logger().warn("No pose data available to log.")

    def send_robot_command(self):
        """Sends pose commands to the robot arm."""
        if self.trajectory_index < len(self.trajectory):
            pose = self.trajectory[self.trajectory_index]
            pose_command = f"set_pose:[{pose[0]},{pose[1]},{pose[2]}];"
            timestamp = time.time() - self.start_time
            msg = String()
            msg.data = pose_command
            self.serial_pub.publish(msg)

            # Log the command
            log_entry = f"{timestamp:.4f},{pose[0]:.4f},{pose[1]:.4f},{pose[2]:.4f}\n"
            self.write_to_log(self.command_log_file, log_entry)
            #self.get_logger().info(f"Sent and logged command: {log_entry.strip()}")
            self.trajectory_index += 1
            self.get_logger().info(f"Sent pose command: {pose_command}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard pressed")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
