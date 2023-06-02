import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    
    def __init__(self):
        super().__init__('EXAMPLE')

def main(args=None):
    rclpy.init(args=args)

    example_node = ExampleNode()

    rclpy.spin(example_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    example_node.destroy_node()
    rclpy.shutdown()

    