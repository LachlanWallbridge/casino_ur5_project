#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from custom_interface.msg import DiceResults
from custom_interface.srv import MovementRequest
from geometry_msgs.msg import Pose


class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        # ---- Clients ----
        self.movement_client = self.create_client(MovementRequest, '/moveit_path_plan')
        while not self.movement_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Movement Service...')

        # ---- Subscribers ----
        self.dice_subscription = self.create_subscription(
            DiceResults,
            '/dice_results',
            self.dice_callback,
            10
        )

        self.latest_dice = []
        self.get_logger().info('Brain node started. Ready to orchestrate the game.')

    # ==============================================================
    #   CALLBACK
    # ==============================================================

    def dice_callback(self, msg: DiceResults):
        """Update list of currently detected dice."""
        self.latest_dice = msg.dice

    # ==============================================================
    #   GAME LOOP
    # ==============================================================

    def run_game(self):
        """Main orchestration loop."""
        while rclpy.ok():
            input("\nPress ENTER to start a new round...")
            self.get_logger().info("Game started. Waiting for dice...")

            # Wait for any dice to appear
            while len(self.latest_dice) == 0 and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.2)

            if not rclpy.ok():
                break

            # Evaluate the roll (sum of current dice)
            dice = self.latest_dice
            values = [d.dice_number for d in dice]
            total = sum(values)
            odd_even = "odd" if total % 2 else "even"
            self.get_logger().info(f"Detected dice: {values} (Total={total}, {odd_even.upper()})")

            # Closed loop pickup: keep removing dice until none left
            while rclpy.ok() and len(self.latest_dice) > 0:
                rclpy.spin_once(self, timeout_sec=0.2)

                # Take first dice from current feed
                current_dice = self.latest_dice[0]
                self.pickup_dice(current_dice)

                # Wait briefly for arm to finish & detection to update
                time.sleep(0.5)
                rclpy.spin_once(self, timeout_sec=0.2)

            self.get_logger().info("All dice removed. Round complete.\n")

    # ==============================================================
    #   DICE PICKUP ROUTINE
    # ==============================================================

    def pickup_dice(self, dice_msg):
        """Move to the dice pose, pick up, and return home."""
        pose: Pose = dice_msg.pose

        self.get_logger().info(
            f"Picking up dice {dice_msg.dice_number} at "
            f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}) "
            f"(confidence={dice_msg.confidence:.2f})"
        )

        # Move above dice
        above_pose = [
            pose.position.x,
            pose.position.y,
            pose.position.z + 0.05,  # 5 cm above
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        if not self.call_move_service('cartesian', above_pose, '1'):
            self.get_logger().warn("Move to dice (above) failed. Skipping.")
            return

        # Return home
        home_pose = [0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0]
        if not self.call_move_service('cartesian', home_pose, '0'):
            self.get_logger().warn("Return to home failed.")

        self.get_logger().info("Pickup complete.")

    # ==============================================================
    #   SERVICE CALL
    # ==============================================================

    def call_move_service(self, command: str, positions: list, constraints_id: str) -> bool:
        """Call the MoveIt path planning service."""
        req = MovementRequest.Request()
        req.command = command
        req.positions = positions
        req.constraints_identifier = constraints_id

        future = self.movement_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error("Movement service call failed.")
            return False


# ==============================================================
#   MAIN
# ==============================================================

def main(args=None):
    rclpy.init(args=args)
    node = Brain()
    try:
        node.run_game()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
