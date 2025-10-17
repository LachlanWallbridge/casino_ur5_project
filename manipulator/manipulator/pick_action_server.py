import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# TODO: Import your custom action here
# from custom_interface.action import MoveArm

class PickActionServer(Node):
    def __init__(self):
        super().__init__('pick_action_server')
        self.get_logger().info('Pick action server started.')
        # TODO: Initialize ActionServer here

def main(args=None):
    rclpy.init(args=args)
    node = PickActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
