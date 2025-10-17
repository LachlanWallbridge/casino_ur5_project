import rclpy
from rclpy.node import Node

class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        self.get_logger().info('Brain node started. Orchestrating the game.')

def main(args=None):
    rclpy.init(args=args)
    node = Brain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
