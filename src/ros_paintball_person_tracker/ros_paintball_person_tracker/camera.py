import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
import cv2

class CameraPublisher(Node):
    
    def __init__(self):
        super().__init__('camera_publisher')
        self.logger = self.get_logger()
        self.logger.info("Camera starting...")
        self.image_pub = self.create_publisher(Image, 'image', 10)
        self.cv_bridge = CvBridge()


        self.cap: cv2.VideoCapture = cv2.VideoCapture(0)
        fps = 20 # frames/second (20 seems to work fine on the laptop!)
        self.timer = self.create_timer(1/fps, self.capture_image_callback)
        self.logger.info("Camera started")

    def capture_image_callback(self):
        success, frame = self.cap.read()
        if success:
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame)
            img_msg.header.stamp = self.get_clock().now().to_msg()
            # frame_id can be ignored currently, since no frame transformations are made
            self.image_pub.publish(img_msg)
        
        #self.logger.info("Publishing video frame")


def main(args=None):
    rclpy.init(args=args)

    cam_node = CameraPublisher()

    rclpy.spin(cam_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cam_node.destroy_node()
    rclpy.shutdown()

    