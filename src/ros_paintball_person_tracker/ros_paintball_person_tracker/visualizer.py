import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from interfaces.msg import YoloResults
import cv2

class VisualizerNode(Node):
    
    def __init__(self):
        super().__init__('visualizer_node')
        self.logger = self.get_logger()
        self.logger.info("Visualizer starting...")
        image_sub = Subscriber(self, Image, "image")
        yolo_sub = Subscriber(self, YoloResults, "yolo_objects")
        self.synchronizer = TimeSynchronizer([image_sub, yolo_sub], 10)
        self.synchronizer.registerCallback(self.yolo_visualize_callback)
        self.cv_bridge = CvBridge()
        self.logger.info("Visualizer started")
    
    def yolo_visualize_callback(self, image_msg, yolo_objects_msg):
        frame = self.cv_bridge.imgmsg_to_cv2(image_msg)

        for yolo_object in yolo_objects_msg.yolo_results:
            box = yolo_object.bbox
            red = (0, 0, 255)
            frame = cv2.rectangle(frame, box[:2], box[2:], color=red, thickness=2)
            # Add class label and confidence score as text
            label = f"{yolo_object.type}: {yolo_object.confidence:.2f}"
            cv2.putText(frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, red, 2)
        cv2.imshow("Yolo Visualization", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    visualizer_node = VisualizerNode()

    rclpy.spin(visualizer_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    visualizer_node.destroy_node()
    rclpy.shutdown()

    