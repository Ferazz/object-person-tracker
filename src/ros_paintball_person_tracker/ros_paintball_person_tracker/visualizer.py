import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from interfaces.msg import YoloResults
import cv2
import math
from typing import Tuple

class VisualizerNode(Node):
    
    def __init__(self):
        super().__init__('visualizer_node')
        self.logger = self.get_logger()
        self.logger.info("Visualizer starting...")
        self.declare_parameter('center_offset')
        self.declare_parameter('radius')
        image_sub = Subscriber(self, Image, "image")
        yolo_sub = Subscriber(self, YoloResults, "yolo_objects")
        self.synchronizer = TimeSynchronizer([image_sub, yolo_sub], 10)
        self.synchronizer.registerCallback(self.yolo_visualize_callback)
        self.cv_bridge = CvBridge()
        self.logger.info("Visualizer started")
    
    def yolo_visualize_callback(self, image_msg, yolo_objects_msg):
        frame = self.cv_bridge.imgmsg_to_cv2(image_msg)
        should_fire = False

        image_center = (image_msg.width//2, image_msg.height//2)
        center_offset: Tuple[int, int] = self.get_parameter('center_offset').get_parameter_value().integer_array_value
        circle_reference_point: Tuple[int, int] = (image_center[0] + center_offset[0], image_center[1] + center_offset[1])
        radius: int = self.get_parameter('radius').get_parameter_value().integer_value

        for yolo_object in yolo_objects_msg.yolo_results:
            box = yolo_object.bbox
            red = (0, 0, 255)
            cv2.rectangle(frame, box[:2], box[2:], color=red, thickness=2)
            # Add class label and confidence score as text
            label = f"{yolo_object.type}: {yolo_object.confidence:.2f}"
            cv2.putText(frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, red, 2)

            bbox_mid_point: Tuple[int, int] = (int(box[0] + box[2])//2, int(box[1] + box[3])//2)
            should_fire = should_fire or calc_distance(bbox_mid_point, circle_reference_point) < radius
            cv2.circle(frame, bbox_mid_point, 2, (0, 0, 0), 2)
        

        if should_fire:
            cv2.circle(frame, circle_reference_point, radius, (0, 255, 0), 3)            
        else:
            cv2.circle(frame, circle_reference_point, radius, (255, 0, 0), 3)
        cv2.imshow("Yolo Visualization", frame)
        cv2.waitKey(1)

def calc_distance(pt1, pt2):
    x_dist = pow(pt1[0] - pt2[0], 2)
    y_dist = pow(pt1[1] - pt2[1], 2)
    return math.sqrt(x_dist + y_dist)
def main(args=None):
    rclpy.init(args=args)

    visualizer_node = VisualizerNode()

    rclpy.spin(visualizer_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    visualizer_node.destroy_node()
    rclpy.shutdown()

    