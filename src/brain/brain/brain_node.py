import rclpy
from rclpy.node import Node
from custom_interface.srv import MovementRequest

class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        # Create a services
        self.movement_client = self.create_client(MovementRequest, '/moveit_path_plan')

        # Wait for the services to be available
        while not self.movement_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Movement Service not available, waiting again...')

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
