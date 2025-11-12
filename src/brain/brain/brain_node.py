#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interface.msg import DiceResults
from custom_interface.srv import StartRound
from custom_interface.action import Movement
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion


class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        # ---- Parameters ----
        self.declare_parameter('manual_start', False)
        self.manual_start = self.get_parameter('manual_start').get_parameter_value().bool_value

        # ---- Action Client ----
        self.movement_action_client = ActionClient(self, Movement, '/moveit_path_plan')
        self.get_logger().info('Waiting for MoveIt Action Server...')
        self.movement_action_client.wait_for_server()
        self.get_logger().info('✅ Connected to MoveIt Action Server.')

        # ---- Subscribers ----
        self.dice_subscription = self.create_subscription(
            DiceResults,
            '/dice_results',
            self.dice_callback,
            10
        )

        # ---- Services (only if not manual) ----
        if not self.manual_start:
            self.start_round_srv = self.create_service(
                StartRound, '/start_round', self.start_round_callback
            )
            self.get_logger().info('🧠 Brain node ready. Awaiting /start_round service calls.')
        else:
            # Run a background thread for console input
            threading.Thread(target=self.wait_for_manual_start, daemon=True).start()
            self.get_logger().info('🧠 Brain node ready. Press ENTER in console to start a round.')

        # ---- State ----
        self.latest_dice = []
        self.round_active = False

    # ============================================================== #
    #   CALLBACKS
    # ============================================================== #

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

        self.start_round()
        response.accepted = True
        response.message = "Round started successfully."
        return response

    # ============================================================== #
    #   MANUAL ROUND START
    # ============================================================== #

    def wait_for_manual_start(self):
        """Blocking console input to manually trigger a round."""
        while rclpy.ok():
            input("\nPress ENTER to start a round...\n")
            if not self.round_active:
                self.get_logger().info("🎯 Manual start detected. Starting new round...")
                self.start_round()
            else:
                self.get_logger().warn("Round already in progress.")

    # ============================================================== #
    #   ROUND EXECUTION
    # ============================================================== #

    def start_round(self):
        """Common entry point to start a round."""
        if self.round_active:
            self.get_logger().warn("Round already running — ignoring start request.")
            return

        self.round_active = True
        self.create_timer(0.1, self._run_round_once)

    def _run_round_once(self):
        """Executes a single round (non-blocking via timer)."""
        for timer in list(self.timers):
            if timer.callback == self._run_round_once:
                timer.cancel()

        self.get_logger().info("Waiting for dice to appear...")

        # Wait up to 10 seconds for dice
        wait_time = 0.0
        while len(self.latest_dice) != 2 and wait_time < 10.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            wait_time += 0.2

        if len(self.latest_dice) != 2:
            self.get_logger().warn(f"{len(self.latest_dice)} dice detected. Ending round.")
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

            # Short delay to allow for updated detections
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().info("✅ Round complete. All dice removed.")
        self.round_active = False

    # ============================================================== #
    #   DICE PICKUP ROUTINE
    # ============================================================== #

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

        if not self.call_move_action('cartesian', pose_rpy, '1'):
            self.get_logger().warn("Move to dice (above) failed. Skipping.")
            return
        
        # TODO: Grip dice (not implemented)

        # Return home
        positions = [-1.3, 1.57, -1.83, -1.57, 0, 0]
        if not self.call_move_action('joint', positions, '0'):
            self.get_logger().warn("Return to home failed.")

        # TODO: Release dice (not implemented)
        
        self.get_logger().info("Pickup complete.")

    # ============================================================== #
    #   ACTION CALL
    # ============================================================== #

    def call_move_action(self, command: str, positions: list, constraints_id: str) -> bool:
        """Send goal to MoveIt action server and wait for result."""
        goal_msg = Movement.Goal()
        goal_msg.command = command
        goal_msg.positions = positions
        goal_msg.constraints_identifier = constraints_id

        self.get_logger().info(f"Sending MoveIt goal: {command} → {positions}")

        # Send goal asynchronously
        send_goal_future = self.movement_action_client.send_goal_async(
            goal_msg
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveIt action server.")
            return False

        self.get_logger().info("Goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future,
            feedback_callback=self.feedback_callback)
        result = get_result_future.result().result

        success = result.success
        if success:
            self.get_logger().info("✅ MoveIt action succeeded.")
        else:
            self.get_logger().warn("❌ MoveIt action failed.")
        return success

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"[MoveIt Feedback] {feedback.status}")


# ============================================================== #
#   MAIN
# ============================================================== #

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
