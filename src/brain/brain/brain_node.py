#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from custom_interface.msg import DiceResults
from custom_interface.srv import MovementRequest, StartRound
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion


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

        # ---- Services ----
        self.start_round_srv = self.create_service(
            StartRound, '/start_round', self.start_round_callback
        )

        # ---- State ----
        self.latest_dice = []
        self.round_active = False

        self.get_logger().info('🧠 Brain node ready. Awaiting start_round calls.')

    # ==============================================================
    #   CALLBACKS
    # ==============================================================

    def dice_callback(self, msg: DiceResults):
        """Update list of currently detected dice."""
        self.latest_dice = msg.dice

    def start_round_callback(self, request, response):
        """Handle frontend 'Start Round' service calls."""
        if not request.start:
            response.accepted = False
            response.message = "Start flag was false — ignoring request."
            return response

        if self.round_active:
            response.accepted = False
            response.message = "Round already in progress."
            return response

        self.round_active = True
        self.get_logger().info("🎯 StartRound service received. Starting a new round...")
        response.accepted = True
        response.message = "Round started successfully."

        # Run the round asynchronously
        self.create_timer(0.1, self._run_round_once)
        return response

    # ==============================================================
    #   ROUND EXECUTION
    # ==============================================================

    def _run_round_once(self):
        """Executes a single round (non-blocking via timer)."""
        # Stop this timer immediately (we only need one tick)
        for timer in list(self.timers):
            if timer.callback == self._run_round_once:
                timer.cancel()

        self.get_logger().info("Waiting for dice to appear...")

        # TODO: ADD ROLLING CODE HERE

        # Wait up to 10 seconds for dice
        wait_time = 0.0
        while len(self.latest_dice) == 0 and wait_time < 10.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            wait_time += 0.2

        if len(self.latest_dice) == 0:
            self.get_logger().warn("No dice detected. Ending round.")
            self.round_active = False
            return

        # Evaluate dice outcome
        dice = self.latest_dice
        values = [d.dice_number for d in dice]
        total = sum(values)
        odd_even = "odd" if total % 2 else "even"
        self.get_logger().info(f"🎲 Dice rolled: {values} (Total={total}, {odd_even.upper()})")

        # Closed-loop dice removal
        while rclpy.ok() and len(self.latest_dice) > 0:
            rclpy.spin_once(self, timeout_sec=0.2)
            current_dice = self.latest_dice[0]
            self.pickup_dice(current_dice)
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info("✅ Round complete. All dice removed.")
        self.round_active = False

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

        above_pose = [
            pose.position.x,
            pose.position.y,
            pose.position.z + 0.05,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

        roll, pitch, yaw = euler_from_quaternion(above_pose[3:])
        pose_rpy = [above_pose[0], above_pose[1], above_pose[2], roll, pitch, yaw]

        if not self.call_move_service('cartesian', pose_rpy, '1'):
            self.get_logger().warn("Move to dice (above) failed. Skipping.")
            return

        # Return home
        positions = [-1.3, 1.57, -1.83, -1.57, 0, 0]
        if not self.call_move_service('joint', positions, '0'):
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
