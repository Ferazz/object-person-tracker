import rclpy
from rlpy.node import Node

from sensor_msgs.msgs import Image
import cv2

class CameraPublisher(Node):
    
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        
        cap = cv2.VideoCapture(0)
        timer_period = 1/30 # seconds
        self.timer = self.create_timer(timer_period, self.capture_image_callback)
    
    def capture_image_callback(self):
        success, frame = self.cap.read()
        msg = Image()


def main(args=None):
    rclpy.init(args=args)

    cam_node = CameraPublisher()

    rclpy.spin(cam_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cam_node.destroy_node()
    rclpy.shutdown()

    