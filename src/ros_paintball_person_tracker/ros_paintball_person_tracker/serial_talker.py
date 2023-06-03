import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import serial

class SerialTalker(Node):
    """
    Might need to run command: $ sudo adduser your_username dialout
    """
    def __init__(self):
        super().__init__('serial_talker')

        PORT = '/dev/ttyACM0'
        BAUDRATE = 115200
        POLL_RATE = 20 # Polls/second
        self.bus = serial.Serial(PORT, BAUDRATE, timeout=1)
        self.bus.reset_input_buffer()
        self.bus.reset_output_buffer()

        self.serial_sub = self.create_subscription(Vector3, 'serial_out/angular_error', self.send_serial_cb, 10)
        self.serial_pub = self.create_publisher(Vector3, 'serial_in', 10)
        #self.serial_poll_timer = self.create_timer(1/ POLL_RATE, self.receive_serial_cb)
    def send_serial_cb(self, msg):
        self.bus.write((f"{msg.x:0>3}\n" + f"{msg.y:0>3}\n").encode('utf-8'))

    def receive_serial_cb(self):
        raise NotImplementedError("Cannot currently receive serial information")

        self.serial_pub.publish()

def main(args=None):
    rclpy.init(args=args)

    serial_talker_node = SerialTalker()

    rclpy.spin(serial_talker_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_talker_node.destroy_node()
    rclpy.shutdown()

    