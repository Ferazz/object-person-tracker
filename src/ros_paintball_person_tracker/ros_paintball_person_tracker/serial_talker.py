import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Empty
from interfaces.msg import ArduinoPackage
import serial
from enum import IntEnum
from typing import List

class AngularErrorPackage(IntEnum):
    X = 0
    Y = 1

class PidUpdatePackage(IntEnum):
    Kp = 0
    Ki = 1
    Kd = 2


class SerialTalker(Node):
    """
    Might need to run command: $ sudo adduser your_username dialout
    """
    def __init__(self):
        super().__init__('serial_talker')
        self.logger = self.get_logger()
        self.logger.info('Starting serial_talker...')
        PORT = '/dev/ttyACM0'
        PORT = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_75834353430351014252-if00'
        BAUDRATE = 115200
        POLL_RATE = 5 # Polls/second
        self.logger.info(f'serial_talker connected to PORT: {PORT}, with baudrate: {BAUDRATE}')
        self.bus = serial.Serial(PORT, BAUDRATE, timeout=1)
        self.bus.reset_input_buffer()
        self.bus.reset_output_buffer()

        self.serial_error_sub = self.create_subscription(ArduinoPackage, 'serial_out/angular_error', self.send_serial_cb, 10)
        #self.serial_fire_sub = self.create_subscription(Empty, 'serial_out/fire', self.send_fire_cb, 10)
        # self.serial_pid_update_sub
        self.serial_pub = self.create_publisher(String, 'serial_in', 10)
        #self.serial_poll_timer = self.create_timer(1/ POLL_RATE, self.receive_serial_cb)

        while not self.bus.is_open:
            #Wait
            ...
        
        self.logger.info('serial_talker started')
    
    def send_serial_cb(self, msg):
        #self.logger.info(f'Received msg: {str(msg)}')
        #data_to_send: str = f'{msg.id:0>3}\n'
        data_to_send = [msg.id]
        
        id = msg.id
        data: List[int] = msg.data
        # Add 'empty' bytes at the end so all packages have the same size
        if id == ArduinoPackage.UNDEFINED:
            return
        # elif id == ArduinoPackage.ANGULAR_ERROR:
        #     data_to_send += f'{data[AngularErrorPackage.X]:0>3}\n' + f'{data[AngularErrorPackage.Y]:0>3}\n' + f'{0:0>3}\n'
        # elif id == ArduinoPackage.FIRE:
        #     data_to_send += f'{0:0>3}\n' + f'{0:0>3}\n' + f'{0:0>3}\n'
        # elif id == ArduinoPackage.PID_UPDATE:
        #     data_to_send += f'{data[PidUpdatePackage.Kp]:0>3}\n' + f'{PidUpdatePackage.Ki:0>3}\n' + f'{PidUpdatePackage.Kd:0>3}\n'
        data_to_send += data

        #self.logger.info(f'Sending package: {data_to_send}')
        self.send_serial(data_to_send)
    
    #def send_fire_cb(self, msg):
    #    self.bus.

    def receive_serial_cb(self):
        serial_received: str = self.bus.readline().decode('utf-8').rstrip()
        if not serial_received:
            return
        self.logger.info(serial_received)
        msg = String()
        msg.data = serial_received
        self.serial_pub.publish(msg)


    def send_serial(self, msg: List):
        """ Send a message on the serial interface
        using [ and ] as start/end markers
        """
        serial_msg_to_send = str(msg) + "\n"
        self.bus.write(serial_msg_to_send.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)

    serial_talker_node = SerialTalker()

    rclpy.spin(serial_talker_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_talker_node.destroy_node()
    rclpy.shutdown()

    