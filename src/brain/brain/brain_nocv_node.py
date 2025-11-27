# #!/usr/bin/env python3
# import threading
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from custom_interface.srv import StartRound
# from custom_interface.action import Movement

# class BrainNode(Node):
#     def __init__(self):
#         super().__init__('brain_node')

#         # Service
#         self.srv = self.create_service(StartRound, '/start_round', self.start_round_callback)

#         # Action client
#         self.action_client = ActionClient(self, Movement, '/moveit_path_plan')

#         # Lock for goal in progress
#         self._lock = threading.Lock()
#         self._goal_in_progress = False

#         # Event to wait for individual motion completion
#         self._motions_done_event = threading.Event()

#         # Motion sequence state
#         self._motions = []
#         self._current_motion_index = 0

#         self.get_logger().info("Brain node started and ready.")

#     # --- Service callback ---
#     def start_round_callback(self, request, response):
#         if not request.start:
#             response.accepted = False
#             response.message = "Start flag is False."
#             return response

#         self.get_logger().info("Service request received: start round")

#         # Start motion sequence in a background thread
#         threading.Thread(target=self._motion_sequence_thread, daemon=True).start()
#         # self._motion_sequence_thread() // This will block the action client to receive goal response

    
#         response.accepted = True
#         response.message = "Round started. Motions running in background."
#         return response

#     # --- Background thread running all motions sequentially ---
#     def _motion_sequence_thread(self):
#         self.get_logger().info("Motion sequence thread started.")

#         # Define motions
#         self._motions = [
#             ("cartesian", [0.4, 0.2, 0.3, 3.1416, 0.0, -1.5708], "FULL"),
#             ("cartesian", [0.4, 0.2, 0.1, 3.1416, 0.0, -1.5708], "FULL"),
#             ("cartesian", [0.4, 0.2, 0.3, 3.1416, 0.0, -1.5708], "FULL"),
#             ("joint",     [-1.5708, 0.7679, -0.7679, -1.5708, 0.0, 0.0], "NONE"),
#         ]

#         for i, (cmd, pos, constr) in enumerate(self._motions):
#             self._current_motion_index = i
#             self.get_logger().info(f"Sending motion {i+1}: {cmd} -> {pos}")

#             # Clear event before sending motion
#             self._motions_done_event.clear()
#             self.send_motion_goal(cmd, pos, constr)

            
#             self._motions_done_event.wait()  # Wait until this motion finishes
#             self.get_logger().info(f"Motion {i+1} finished")

#         self.get_logger().info("🎯 All motions completed.")


#     # --- Send one motion goal ---
#     def send_motion_goal(self, command, positions, constraint):
#         with self._lock:
#             if self._goal_in_progress:
#                 self.get_logger().warn("Goal already in progress, skipping...")
#                 return
#             self._goal_in_progress = True

#         goal_msg = Movement.Goal()
#         goal_msg.command = command
#         goal_msg.positions = positions
#         goal_msg.constraints_identifier = constraint

#         # Wait for server
#         if not self.action_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error("Action server not available after waiting.")
#             # set _motions_done_event to so .wait() stop waiting
#             self._motions_done_event.set() 
#             return

#         future = self.action_client.send_goal_async(
#             goal_msg, feedback_callback=self.feedback_callback
#         )
#         future.add_done_callback(self.goal_response_callback)

#     # --- Action callbacks ---
#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("Goal rejected by server.")
#             with self._lock:
#                 self._goal_in_progress = False
#             self._motions_done_event.set()
#             return

#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.result_callback)

#     def feedback_callback(self, feedback_msg):
#         status = getattr(feedback_msg.feedback, "status", "<no status>")
#         self.get_logger().info(f"Feedback: {status}")

#     def result_callback(self, future):
#         result = future.result().result
#         motion_num = self._current_motion_index + 1

#         if getattr(result, "success", False):
#             self.get_logger().info(f"✅ Motion {motion_num} succeeded.")
#         else:
#             self.get_logger().warn(f"⚠️ Motion {motion_num} failed.")

#         with self._lock:
#             self._goal_in_progress = False

#         # Signal that this motion is done
#         self._motions_done_event.set()

#     def destroy_node(self):
#         self.get_logger().info("Shutting down BrainNode...")
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = BrainNode()
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node, timeout_sec=0.1)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3
import threading
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from custom_interface.srv import StartRound
from custom_interface.action import Movement

from sensor_msgs.msg import JointState


class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')

        # Allow callbacks to run concurrently
        self.cb_group = ReentrantCallbackGroup()

        # --- Service ---
        self.srv = self.create_service(
            StartRound,
            '/start_round',
            self.start_round_callback,
            callback_group=self.cb_group
        )

        # --- Action client ---
        self.action_client = ActionClient(
            self,
            Movement,
            '/moveit_path_plan',
            callback_group=self.cb_group
        )

        # --- Joint states (continuous, but cheap) ---
        self._joint_lock = threading.Lock()
        self._last_joint_state = None
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.cb_group
        )

        # --- Thread sync ---
        self._lock = threading.Lock()
        self._goal_in_progress = False

        self._motions_done_event = threading.Event()
        self._action_done_event = threading.Event()

        # Motion state
        self._motions = []
        self._current_motion_index = 0

        self.get_logger().info("Brain node ready.")

    # =============================================================
    # JOINT STATE CALLBACK (VERY LIGHTWEIGHT)
    # =============================================================
    def joint_state_callback(self, msg):
        # just store latest; no logging, no extra work
        with self._joint_lock:
            self._last_joint_state = msg

    # =============================================================
    # SERVICE CALLBACK
    # =============================================================
    def start_round_callback(self, request, response):
        if not request.start:
            response.accepted = False
            response.message = "Start flag is False."
            return response

        self.get_logger().info("Service: START ROUND received")

        # Prepare wait
        self._action_done_event.clear()

        # Start motion sequence thread
        threading.Thread(
            target=self._motion_sequence_thread,
            daemon=True
        ).start()

        # SAFELY wait for sequence completion
        self.get_logger().info("Waiting for motion sequence to finish...")
        self._action_done_event.wait()
        self.get_logger().info("Sequence finished, responding.")

        response.accepted = True
        response.message = "Round completed successfully."
        return response

    # =============================================================
    # MOTION SEQUENCE THREAD
    # =============================================================
    def _motion_sequence_thread(self):
        self.get_logger().info("Motion sequence thread started.")

        # ----------------------------------------------
        # Original motions
        # ----------------------------------------------
        self._motions = [
            ("cartesian", [0.4, 0.2, 0.3, math.pi/2, 0.0, -math.pi], "FULL WRITST1"),  # 0
            ("cartesian", [0.4, 0.2, 0.1, math.pi/2, 0.0, -math.pi], "FULL WRITST1"),  # 1
            ("cartesian", [0.4, 0.2, 0.3, math.pi/2, 0.0, -math.pi], "FULL WRITST1"),  # 2
            ("joint",     [-1.5708, 0.7679, -0.7679, -1.5708, 0.0, 0.0], "NONE"),     # 3 (will become 4)
        ]

        # ----------------------------------------------
        # Insert new custom motion at index 3
        # ----------------------------------------------
        self._motions.insert(3, ("custom_wrist3", None, None))  # now index 3

        # ----------------------------------------------
        # Execute motions in order
        # ----------------------------------------------
        for i, (cmd, pos, constr) in enumerate(self._motions):
            self._current_motion_index = i
            self.get_logger().info(f"Executing motion {i}: {cmd}")

            # ------------------------------------------
            # NEW CUSTOM MOTION AT INDEX 3
            # ------------------------------------------
            if cmd == "custom_wrist3":
                self.get_logger().info("📌 NEW MOTION: Rotate wrist_3_joint by +120°")

                # Wait for valid joint state
                js = None
                while js is None:
                    with self._joint_lock:
                        js = self._last_joint_state
                    if js is None:
                        rclpy.sleep(0.1)

                current_joints = list(js.position)

                # +120° = +2π/3
                delta = 2.0 * math.pi / 3.0
                current_joints[4] += delta

                self.get_logger().info(f"New wrist_3_joint = {current_joints[4]:.3f} rad")

                # Clear event
                self._motions_done_event.clear()

                # Send as joint goal
                self.send_motion_goal("joint", current_joints, "NONE")

                # Wait
                self._motions_done_event.wait()
                self.get_logger().info("Custom wrist motion complete.")
                continue

            # ------------------------------------------
            # NORMAL MOTION
            # ------------------------------------------
            self._motions_done_event.clear()
            self.send_motion_goal(cmd, pos, constr)
            self._motions_done_event.wait()

            self.get_logger().info(f"Motion {i} complete.")

        # ----------------------------------------------
        # All done
        # ----------------------------------------------
        self.get_logger().info("🎯 ALL motions completed.")
        self._action_done_event.set()


    # =============================================================
    # SEND GOAL
    # =============================================================
    def send_motion_goal(self, command, positions, constraint):
        with self._lock:
            if self._goal_in_progress:
                self.get_logger().warn("Goal already running, skipping.")
                return
            self._goal_in_progress = True

        # Build goal
        goal = Movement.Goal()
        goal.command = command
        goal.positions = positions
        goal.constraints_identifier = constraint

        # Check server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server unavailable.")
            with self._lock:
                self._goal_in_progress = False
            self._motions_done_event.set()
            self._action_done_event.set()
            return

        # Send async
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    # =============================================================
    # ACTION CALLBACKS
    # =============================================================
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            with self._lock:
                self._goal_in_progress = False
            self._motions_done_event.set()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        status = getattr(feedback_msg.feedback, "status", "<no status>")
        self.get_logger().info(f"Feedback: {status}")

    def result_callback(self, future):
        result = future.result().result
        motion_num = self._current_motion_index

        if getattr(result, "success", False):
            self.get_logger().info(f"✅ Motion {motion_num} succeeded.")
        else:
            self.get_logger().warn(f"⚠️ Motion {motion_num} failed.")

        with self._lock:
            self._goal_in_progress = False

        self._motions_done_event.set()

    def destroy_node(self):
        self.get_logger().info("Shutting down BrainNode...")
        super().destroy_node()


# ======================================================================
# MAIN WITH MULTITHREADED EXECUTOR
# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
