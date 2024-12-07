import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.tools.list_ports


class RobotServer(Node):
    def __init__(self, name='robot_server', port='/dev/ttyACM0', baudrate=9600, verbose=False):
        """ ROS2 node that sends and receives messages to/from a serial port

        Parameters:
        -----------
        name : (str) Name of the ROS2 node
        port : (str) Serial port to connect to
        baudrate : (int) Baudrate of the serial connection
        verbose : (bool) Enable/disable verbose debugging output
        """

        super().__init__(name)
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.serial = None

        self.declare_parameter('port', port)
        self.declare_parameter('baudrate', baudrate)
        self.declare_parameter('serial_command_topic', '/mini_arm/input/serial_message')
        self.declare_parameter('robot_joint_state_topic', '/mini_arm/output/joint_states')

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'Successfully connected to serial port {self.port}')
            self.connected = True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')

        # Create subscription to the serial topic
        topic = self.get_parameter('serial_command_topic').get_parameter_value().string_value
        self.subscription = self.create_subscription(String, topic, self.serial_message_callback, 10)

        # Create timer to read from serial port for any incoming messages
        self.timer = self.create_timer(0.05, self.serial_read_callback)

    def serial_message_callback(self, msg):
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(msg.data.encode('utf-8'))
                self.get_logger().info(f'Sent: {msg.data}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error sending to serial: {e}')
        else:
            self.get_logger().error('Serial connection not open')

    def serial_read_callback(self):
        if self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    received_data = self.serial.readline().decode('utf-8').strip()
                    self.get_logger().info(f'{received_data}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error reading from serial: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
