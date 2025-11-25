# #!/usr/bin/env python3
# import time
# import threading
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from custom_interface.msg import DiceResults
# from custom_interface.srv import StartRound
# from custom_interface.action import Movement
# from geometry_msgs.msg import Pose
# from tf_transformations import euler_from_quaternion
# from rclpy.executors import MultiThreadedExecutor



# class Brain(Node):
#     def __init__(self):
#         super().__init__('brain')

#         # ---- Parameters ----
#         self.declare_parameter('manual_start', False)
#         self.manual_start = self.get_parameter('manual_start').get_parameter_value().bool_value

#         # ---- Action Client ----
#         self.movement_action_client = ActionClient(self, Movement, '/moveit_path_plan')
#         self.get_logger().info('Waiting for MoveIt Action Server...')
#         self.movement_action_client.wait_for_server()
#         self.get_logger().info('✅ Connected to MoveIt Action Server.')

#         # ---- Subscribers ----
#         self.dice_subscription = self.create_subscription(
#             DiceResults,
#             '/dice_results',
#             self.dice_callback,
#             10
#         )

#         # ---- Services (only if not manual) ----
#         if not self.manual_start:
#             self.start_round_srv = self.create_service(
#                 StartRound, '/start_round', self.start_round_callback
#             )
#             self.get_logger().info('🧠 Brain node ready. Awaiting /start_round service calls.')
#         else:
#             # Run a background thread for console input
#             threading.Thread(target=self.wait_for_manual_start, daemon=True).start()
#             self.get_logger().info('🧠 Brain node ready. Press ENTER in console to start a round.')

#         # ---- State ----
#         self.latest_dice = []
#         self.round_active = False

#     # ============================================================== #
#     #   CALLBACKS
#     # ============================================================== #

#     def dice_callback(self, msg: DiceResults):
#         """Update list of currently detected dice."""
#         self.latest_dice = msg.dice

#     def start_round_callback(self, request, response):
#         """Handle frontend 'Start Round' service calls."""
#         if not request.start:
#             response.accepted = False
#             response.message = "Start flag was false — ignoring request."
#             return response

#         if self.round_active:
#             response.accepted = False
#             response.message = "Round already in progress."
#             return response

#         self.start_round()
#         response.accepted = True
#         response.message = "Round started successfully."
#         return response

#     # ============================================================== #
#     #   MANUAL ROUND START
#     # ============================================================== #

#     def wait_for_manual_start(self):
#         """Blocking console input to manually trigger a round."""
#         while rclpy.ok():
#             input("\nPress ENTER to start a round...\n")
#             if not self.round_active:
#                 self.get_logger().info("🎯 Manual start detected. Starting new round...")
#                 self.start_round()
#             else:
#                 self.get_logger().warn("Round already in progress.")

#     # ============================================================== #
#     #   ROUND EXECUTION
#     # ============================================================== #

#     def start_round(self):
#         """Common entry point to start a round."""
#         # if self.round_active:
#         #     self.get_logger().warn("Round already running — ignoring start request.")
#         #     return

#         self.round_active = True
#         self.create_timer(0.1, self._run_round_once)

#     def _run_round_once(self):
#         """Executes a single round (non-blocking via timer)."""
#         for timer in list(self.timers):
#             if timer.callback == self._run_round_once:
#                 timer.cancel()

#         self.get_logger().info("Waiting for dice to appear...")

#         # Wait up to 10 seconds for dice
#         wait_time = 0.0
#         while len(self.latest_dice) != 2 and wait_time < 10.0 and rclpy.ok():
#             rclpy.spin_once(self, timeout_sec=0.2)
#             wait_time += 0.2

#         if len(self.latest_dice) != 2:
#             self.get_logger().warn(f"{len(self.latest_dice)} dice detected. Ending round.")
#             self.round_active = False
#             return

#         # Evaluate dice outcome
#         dice = self.latest_dice
#         values = [d.dice_number for d in dice]
#         total = sum(values)
#         odd_even = "odd" if total % 2 else "even"
#         self.get_logger().info(f"🎲 Dice rolled: {values} (Total={total}, {odd_even.upper()})")

#         # Closed-loop dice removal
#         while rclpy.ok() and len(self.latest_dice) > 0:
#             rclpy.spin_once(self, timeout_sec=0.2)
#             current_dice = self.latest_dice[0]
#             self.pickup_dice(current_dice)

#             # Short delay to allow for updated detections
#             time.sleep(0.5)
#             rclpy.spin_once(self, timeout_sec=0.2)

#         self.get_logger().info("✅ Round complete. All dice removed.")
#         self.round_active = False

#     # ============================================================== #
#     #   DICE PICKUP ROUTINE
#     # ============================================================== #

#     def pickup_dice(self, dice_msg):
#         pose = dice_msg.pose

#         above_pose = [
#             pose.position.x,
#             pose.position.y,
#             pose.position.z + 0.05,
#             pose.orientation.x,
#             pose.orientation.y,
#             pose.orientation.z,
#             pose.orientation.w,
#         ]

#         roll, pitch, yaw = euler_from_quaternion(above_pose[3:])
#         pose_rpy = [above_pose[0], above_pose[1], above_pose[2], roll, pitch, yaw]

#         # 1️⃣ Move above dice
#         if self.call_move_action('cartesian', pose_rpy, '1'):
#             self.get_logger().info("Reached above dice. Proceeding to pickup.")
#         else:
#             self.get_logger().error("Failed to reach above dice.")
#             return

#         # 2️⃣ Return home (only after success)
#         home_joints = [-1.3, 1.57, -1.83, -1.57, 0, 0]
#         self.call_move_action('joint', home_joints, '0')


#     # ============================================================== #
#     #   ACTION CALL
#     # ============================================================== #

#     def call_move_action(self, command: str, positions: list, constraints_id: str) -> bool:
#         """Send goal to MoveIt action server and wait for result."""
#         goal_msg = Movement.Goal()
#         goal_msg.command = command
#         goal_msg.positions = positions
#         goal_msg.constraints_identifier = constraints_id

#         self.get_logger().info(f"Sending MoveIt goal: {command} → {positions}")

#         # Send goal asynchronously
#         send_goal_future = self.movement_action_client.send_goal_async(goal_msg)
#         self.wait_for_future(send_goal_future)
#         goal_handle = send_goal_future.result()

#         if not goal_handle.accepted:
#             self.get_logger().error("Goal rejected by MoveIt action server.")
#             return False

#         self.get_logger().info("Goal accepted. Waiting for result...")
#         get_result_future = goal_handle.get_result_async()
#         rclpy.spin_until_future_complete(self, get_result_future)
#         result = get_result_future.result().result
#         success = result.success
#         if success:
#             self.get_logger().info("✅ MoveIt action succeeded.")
#         else:
#             self.get_logger().warn("❌ MoveIt action failed.")
#         return success

#     # def call_move_action(self, command: str, positions: list, constraints_id: str):
#     #     goal_msg = Movement.Goal()
#     #     goal_msg.command = command
#     #     goal_msg.positions = positions
#     #     goal_msg.constraints_identifier = constraints_id

#     #     self.get_logger().info(f"Sending MoveIt goal: {command} → {positions}")

#     #     send_goal_future = self.movement_action_client.send_goal_async(
#     #         goal_msg, feedback_callback=self.feedback_callback
#     #     )
#     #     # Wait for the goal to be accepted/rejected
#     #     rclpy.spin_until_future_complete(self, send_goal_future)
#     #     goal_handle = send_goal_future.result()

#     #     if not goal_handle.accepted:
#     #         self.get_logger().error("Goal rejected by MoveIt action server.")
#     #         return False

#     #     self.get_logger().info("Goal accepted. Waiting for result...")

#     #     # Wait for result
#     #     get_result_future = goal_handle.get_result_async()
#     #     rclpy.spin_until_future_complete(self, get_result_future)
#     #     result = get_result_future.result()

#     #     if result.status == GoalStatus.STATUS_SUCCEEDED:
#     #         self.get_logger().info("MoveIt action succeeded.")
#     #         return True
#     #     else:
#     #         self.get_logger().warn(f"MoveIt action failed with status: {result.status}")
#     #         return False


#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("Goal rejected by MoveIt action server.")
#             return

#         self.get_logger().info("Goal accepted. Waiting for result...")
#         get_result_future = goal_handle.get_result_async()
#         get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         if result.success:
#             self.get_logger().info("✅ MoveIt action succeeded.")
#         else:
#             self.get_logger().warn("❌ MoveIt action failed.")


#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         self.get_logger().info(f"[MoveIt Feedback] {feedback.status}")



#     def wait_for_future(self, future):
#         """Spin the node until the future is complete, processing callbacks."""
#         while rclpy.ok() and not future.done():
#             rclpy.spin_once(self, timeout_sec=0.1)



# # ============================================================== #
# #   MAIN
# # ============================================================== #

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = Brain()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()


# # if __name__ == "__main__":
# #     main()

# def main(args=None):
#     rclpy.init(args=args)
#     node = Brain()
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node, timeout_sec=0.1)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interface.msg import DiceResults
from custom_interface.srv import StartRound, GripperCmd
from custom_interface.action import Movement
from tf_transformations import euler_from_quaternion


class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        # ---- Params ----
        self.declare_parameter('manual_start', False)
        self.manual_start = self.get_parameter('manual_start').get_parameter_value().bool_value

        # ---- Action Client ----
        self.action_client = ActionClient(self, Movement, '/moveit_path_plan')

        # Threading helpers
        self._lock = threading.Lock()
        self._goal_in_progress = False
        self._motion_done_event = threading.Event()

        # ---- Dice subscriber ----
        self.latest_dice = []
        self.create_subscription(DiceResults, '/dice_results', self.dice_callback, 10)

        # ---- Service ----
        if not self.manual_start:
            self.start_srv = self.create_service(StartRound, '/start_round', self.start_round_callback)
            self.get_logger().info("🧠 Brain ready — waiting for /start_round")
        else:
            threading.Thread(target=self.manual_trigger_thread, daemon=True).start()
            self.get_logger().info("🧠 Brain ready — press ENTER to start manually.")

        self.round_active = False

        # ---- Gripper Service ----
        self.gripper_client = self.create_client(GripperCmd, 'gripper_cmd')
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gripper Service...')
        self.get_logger().info('✅ Connected to Gripper Service.')

    # ============================================================== #
    #   CALLBACKS
    # ============================================================== #

    # ================================================================
    #   CALLBACKS
    # ================================================================
    def dice_callback(self, msg):
        self.latest_dice = msg.dice


    # ================================================================
    #   SERVICE CALLBACK (Migrated from No-CV version)
    # ================================================================
    def start_round_callback(self, request, response):
        if not request.start:
            response.accepted = False
            response.message = "Start flag is false."
            return response

        if self.round_active:
            response.accepted = False
            response.message = "Round already running."
            return response

        self.round_active = True
        threading.Thread(target=self.round_thread, daemon=True).start()

        response.accepted = True
        response.message = "Round started."
        return response

    def gripper_command(self, width: int) -> bool:
        """Send command to gripper via service."""
        req = GripperCmd.Request()
        req.width = width

        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        time.sleep(5.0)
        
        if res.success:
            self.get_logger().info(f"Gripper set to width {width}.")
        else:
            self.get_logger().warn(f"Gripper command failed: {res.message}")

        return res.success
    # ============================================================== #
    #   MANUAL ROUND START
    # ============================================================== #

    # ================================================================
    #   MANUAL START THREAD
    # ================================================================
    def manual_trigger_thread(self):
        while rclpy.ok():
            input("\nPress ENTER to start round…\n")
            if not self.round_active:
                self.round_active = True
                threading.Thread(target=self.round_thread, daemon=True).start()
            else:
                self.get_logger().warn("Round already active.")


    # ================================================================
    #   ROUND LOGIC (unchanged, but running in thread)
    # ================================================================
    def round_thread(self):
        self.get_logger().info("Waiting for dice…")

        # Wait up to 10 seconds for 2 dice
        t = 0.0
        while len(self.latest_dice) != 2 and t < 100.0:
            time.sleep(0.2)
            t += 0.2

        if len(self.latest_dice) != 2:
            self.get_logger().warn(f"Detected {len(self.latest_dice)} dice. Cancelling round.")
            self.round_active = False
            return

        dice = self.latest_dice
        values = [d.dice_number for d in dice]
        total = sum(values)

        self.get_logger().info(f"🎲 Dice rolled {values}, total={total} ({'even' if total%2==0 else 'odd'})")

        # Closed-loop pickup
        while rclpy.ok() and len(self.latest_dice) > 0:
            d = self.latest_dice[0]
            self.pickup_dice(d)
            self.get_logger().info("completed action")

            time.sleep(1.5)
            self.get_logger().info(f"{len(self.latest_dice)} dice remaining.")

        self.get_logger().info("✅ Round complete.")
        self.round_active = False


    # ================================================================
    #   DICE PICKUP
    # ================================================================
    def pickup_dice(self, dice_msg):
        self.get_logger().info(f"Picking up dice with value {dice_msg.dice_number}")
        pose = dice_msg.pose

        dice = [
            pose.position.x,
            pose.position.y,
            pose.position.z + 0.115,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        above = [
            pose.position.x,
            pose.position.y,
            pose.position.z + 0.14,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        roll, pitch, yaw = euler_from_quaternion(above[3:])

        above_pose = [above[0], above[1], above[2], roll, pitch, yaw]
        dice_pose = [above[0], above[1], above[2], roll, pitch, yaw]

        # Move above dice
        # Clear event before sending motion
        self._motion_done_event.clear()
        self.send_motion("cartesian", above_pose, "FULL")

        # Wait here in background thread (safe!)
        self._motion_done_event.wait()  # Wait until this motion finishes
        self.get_logger().info(f"Dice motion finished")


        # Move to dice
        # Clear event before sending motion
        self._motion_done_event.clear()
        self.send_motion("cartesian", dice_pose, "FULL")

        # Wait here in background thread (safe!)
        self._motion_done_event.wait()  # Wait until this motion finishes
        self.get_logger().info(f"Dice motion finished")

        # Home        
        if not self.gripper_command(180):  # Grip Width
            self.get_logger().warn("Gripper command failed. Skipping pickup.")
        else:
            self.get_logger().info("Dice gripped successfully.")

        # Return home
         # Clear event before sending motion
        self._motion_done_event.clear()
        self.send_motion("joint", [-1.5708, 0.7679, -0.7679, -1.5708, 0.0, 0.0], "NONE")

        # Wait here in background thread (safe!)
        self._motion_done_event.wait()  # Wait until this motion finishes
        self.get_logger().info(f"Home motion finished")

    
        if not self.gripper_command(40):  # Open gripper
            self.get_logger().warn("Gripper release command failed.")
        else:
            self.get_logger().info("Release complete.")
       


        # # Step 1: move above dice
        # if not self.send_motion("cartesian", cart_pose, "1"):
        #     self.get_logger().error("Failed to reach above dice.")
        #     return

        # # Step 2: return home
        # home = [-1.3, 1.57, -1.83, -1.57, 0.00, 0.00]
        # self.send_motion("joint", home, "0")


    # ================================================================
    #   ACTION CLIENT (Migrated from No-CV version)
    # ================================================================
    def send_motion(self, command, positions, constraint):
        with self._lock:
            if self._goal_in_progress:
                self.get_logger().warn("Goal already in progress.")
                return False
            self._goal_in_progress = True

        self._motion_done_event.clear()

        goal = Movement.Goal()
        goal.command = command
        goal.positions = positions
        goal.constraints_identifier = constraint

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Action server unavailable.")
            self._motion_done_event.set()
            return False

        future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

        # Wait here until result callback signals completion
        self._motion_done_event.wait()
        return True


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            with self._lock:
                self._goal_in_progress = False
            self._motion_done_event.set()
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)


    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("✅ Motion succeeded.")
        else:
            self.get_logger().warn("❌ Motion failed.")

        with self._lock:
            self._goal_in_progress = False

        self._motion_done_event.set()


    def feedback_callback(self, feedback_msg):
        fb = getattr(feedback_msg.feedback, "status", "<no status>")
        self.get_logger().info(f"[Feedback] {fb}")


# ================================================================
#   MAIN
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = Brain()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
