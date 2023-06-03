import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from interfaces.msg import YoloResults
import math
from typing import Tuple

def calc_distance(pt1, pt2):
    x_dist = pow(pt1[0] - pt2[0], 2)
    y_dist = pow(pt1[1] - pt2[1], 2)
    return math.sqrt(x_dist + y_dist)

class RobotController(Node):
    
    def __init__(self):
        super().__init__('robot_controller')
        self.logger = self.get_logger()
        self.logger.info('Starting robot controller...')
        self.declare_parameter('center_offset')
        self.declare_parameter('radius')
        self.yolo_sub = self.create_subscription(YoloResults, 'yolo_objects', self.yolo_cb, 10)
        self.serial_pub = self.create_publisher(Vector3, 'serial_out/angular_error', 10)
        self.DEGREE_PER_PIXEL = 0.1
        self.logger.info('Robot controller started')

    def yolo_cb(self, msg):
        
        image_center = [msg.source_width//2, msg.source_height//2]
        person_bbox_mid_points = []
        for result in msg.yolo_results:
            if result.type == "person":
                box = result.bbox
                bbox_mid_point: Tuple[int, int] = (int(box[0] + box[2])//2, int(box[1] + box[3])//2)
                person_bbox_mid_points.append(bbox_mid_point)


        if person_bbox_mid_points:
            # Currently only use first element
            person_mid_point = person_bbox_mid_points[0]
            
            center_offset: Tuple[int, int] = self.get_parameter('center_offset').get_parameter_value().integer_array_value
            circle_reference_point: Tuple[int, int] = (image_center[0] + center_offset[0], image_center[1] + center_offset[1])
            radius: int = self.get_parameter('radius').get_parameter_value().integer_value
            
             # Positive x error => Aim right
            # Positive y error => Aim up
            x_pixel_error: int = person_mid_point[0] - image_center[0]
            y_pixel_error: int = image_center[1] - person_mid_point[1]

            angular_error_msg = Vector3()
            angular_error_msg.x = x_pixel_error * self.DEGREE_PER_PIXEL
            angular_error_msg.y = y_pixel_error * self.DEGREE_PER_PIXEL
            self.serial_pub.publish(angular_error_msg)
            
            should_fire = calc_distance(circle_reference_point, person_mid_point) < radius

            if should_fire:
                ...
               
    
def main(args=None):
    rclpy.init(args=args)

    robot_controller_node = RobotController()

    rclpy.spin(robot_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller_node.destroy_node()
    rclpy.shutdown()

    