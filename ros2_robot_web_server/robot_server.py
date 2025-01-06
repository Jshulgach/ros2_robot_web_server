
# Python imports
import time
import serial

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class RobotServer(Node):
    def __init__(self, name='robot_server', verbose=False):
        """ ROS2 node that sends and receives messages to/from a serial port

        Parameters:
        -----------
        name : (str) Name of the ROS2 node
        port : (str) Serial port to connect to
        baudrate : (int) Baudrate of the serial connection
        verbose : (bool) Enable/disable verbose debugging output
        """
        super().__init__(name)
        self.connected = False
        self.serial = None
        self.verbose = verbose

        # Initialize ROS2 parameters, publishers, and subscriptions
        self.init_parameters()
        self.init_publishers()
        self.init_subscriptions()

        # Create timer to read from serial port for any incoming messages and sent joint state requests to publish
        #self.read_timer = self.create_timer(0.05, self.read_serial_cb)
        self.joint_state_timer = self.create_timer(self.joint_state_publish_rate, self.joint_state_cb)

        # Attempt to connect to the serial port
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'Successfully connected to serial port {self.port}')
            # Try sending the Ctrl+D character to restart the pico in case it isn't automatically started
            self.serial.write(b'\x04')
            time.sleep(0.1)
            self.serial.write(b'\x04')
            time.sleep(0.1)

            # Set debug mode to off
            self.serial.write(b'debug:false;')
            time.sleep(0.1)

            # flush the serial buffer
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            self.connected = True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')

    def init_subscriptions(self):
        # Create subscription to the serial command topic
        self.subscription = self.create_subscription(String, self.command_topic, self.send_message_cb, 10)

    def init_publishers(self):
        # Create publisher for the robot joint states
        self.joint_state_publisher = self.create_publisher(JointState, self.joint_state_topic, 1)

    def init_parameters(self):
        # Declare ROS2 parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('serial_command_topic', '/robot_server/input/serial_message')
        self.declare_parameter('robot_joint_state_topic', '/robot_server/output/joint_states')
        self.declare_parameters('joint_state_publish_rate', 1.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.command_topic = self.get_parameter('serial_command_topic').get_parameter_value().string_value
        self.joint_state_topic = self.get_parameter('robot_joint_state_topic').get_parameter_value().string_value
        self.joint_state_publish_rate = self.get_parameter('joint_state_publish_rate').get_parameter_value().double_value

    def send_message_cb(self, msg, wait_response=False):
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(msg.data.encode('utf-8'))
                if self.verbose: self.get_logger().info(f'Sent message to serial: {msg.data}')
                if wait_response:
                    received_data = self.serial.readline().decode('utf-8').strip()
                    if self.verbose: self.get_logger().info(f'Received from serial: {received_data}')
                    return received_data
            except serial.SerialException as e:
                self.get_logger().error(f'Error sending to serial: {e}')
        else:
            self.get_logger().error('Serial connection not open')

    def read_serial_cb(self):
        if self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    received_data = self.serial.readline().decode('utf-8').strip()
                    self.get_logger().info(f'Received from serial: {received_data}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error reading from serial: {e}')

    def joint_state_cb(self):
        # Send a request to the robot to publish its joint states
        msg = self.serial_message(String(data='get_joints;'))
        # [594562.750][Controller][0, 0, 0, 0, 0, 0]
        # Get the joint state from the string
        joint_state = msg.split('[')[1].split(']')[0].split(',')

        # Build the JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_state_msg.position = [float(joint) for joint in joint_state]
        joint_state_msg.velocity = [0.0] * 6
        joint_state_msg.effort = [0.0] * 6

        # Publish the joint state message
        self.joint_state_publisher.publish(joint_state_msg)



def main(args=None):
    rclpy.init(args=args)
    node = RobotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
