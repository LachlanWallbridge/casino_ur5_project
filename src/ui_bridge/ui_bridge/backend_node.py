import rclpy
from rclpy.node import Node
from flask import Flask, jsonify

app = Flask(__name__)

@app.route('/players', methods=['GET'])
def get_players():
    # TODO: Load players from DB
    return jsonify([])

class UIBridge(Node):
    def __init__(self):
        super().__init__('ui_bridge')
        self.get_logger().info('UI Bridge node started.')

def main(args=None):
    rclpy.init(args=args)
    node = UIBridge()
    try:
        # Run Flask in a separate thread if needed
        from threading import Thread
        t = Thread(target=app.run, kwargs={'port':5000})
        t.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
