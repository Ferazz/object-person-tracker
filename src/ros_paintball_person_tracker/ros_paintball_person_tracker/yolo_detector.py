import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces.msg import YoloResult, YoloResults

import ultralytics.yolo.engine.results
from ultralytics import YOLO
from typing import List

class YoloDetector(Node):
    
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.logger = self.get_logger()
        self.logger.info("Yolo node starting...")
        self.image_sub = self.create_subscription(Image, 'image', self.image_callback, 10)
        self.yolo_results_pub = self.create_publisher(YoloResults, 'yolo_objects', 10)
        self.cv_bridge = CvBridge()
        self.model = YOLO("/models/yolov8n.pt")
        self.logger.info("Yolo node started")
    
    def image_callback(self, msg):
        frame = self.cv_bridge.imgmsg_to_cv2(msg)
        height, width = frame.shape[:2]

        results: List[ultralytics.yolo.engine.results.Results] = self.model.predict(source=frame, verbose=False)
        
        all_yolo_results_msg = YoloResults()
        all_yolo_results_msg.source_width = width
        all_yolo_results_msg.source_height = height
        all_yolo_results_msg.header = msg.header
        # Results always have length 1?
        for result in results:
            # names is a dictionary with all classifiers and their id's 
            names = result.names
            boxes = result.boxes
            
            if boxes:
                for box, cls, conf in zip(boxes.xyxy, boxes.cls, boxes.conf):
                    # cls is tensor[0.] so needs to be turned into integer number for indexing
                    yolo_result = YoloResult()
                    yolo_result.id = int(cls)
                    yolo_result.type = names[int(cls)]
                    yolo_result.bbox = [int(num) for num in box]
                    yolo_result.confidence = float(conf)

                    all_yolo_results_msg.yolo_results.append(yolo_result)
        self.yolo_results_pub.publish(all_yolo_results_msg)


def main(args=None):
    rclpy.init(args=args)

    yolo_node = YoloDetector()

    rclpy.spin(yolo_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_node.destroy_node()
    rclpy.shutdown()

    