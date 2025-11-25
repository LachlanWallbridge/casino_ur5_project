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

#         self.srv = self.create_service(StartRound, '/start_round', self.start_round_callback)
#         self.action_client = ActionClient(self, Movement, '/moveit_path_plan')

#         self._lock = threading.Lock()
#         self._goal_in_progress = False
#         self._motions_done_event = threading.Event()

#         self._motions = []
#         self._current_motion_index = 0

#         self.get_logger().info("Brain node started and ready.")

#     def start_round_callback(self, request, response):
#         if not request.start:
#             response.accepted = False
#             response.message = "Start flag is False."
#             return response

#         self.get_logger().info("Service request received: start round")

#         # Reset motion event
#         self._motions_done_event.clear()

#         # Launch motions asynchronously
#         threading.Thread(target=self.send_motion_sequence, daemon=True).start()

#         # Wait here until all motions complete
#         self.get_logger().info("Waiting for motion sequence to complete...")
#         completed = self._motions_done_event.wait(timeout=180.0)  # 3 min timeout (optional)

#         if completed:
#             response.accepted = True
#             response.message = "Round completed successfully."
#         else:
#             response.accepted = False
#             response.message = "Timeout waiting for motion sequence."

#         return response


#     def send_motion_sequence(self):
#         self._motions = [
#             ("cartesian", [0.4, 0.2, 0.3, 3.1416, 0.0, -1.5708], "FULL"),
#             ("cartesian", [0.4, 0.2, 0.1, 3.1416, 0.0, -1.5708], "FULL"),
#             ("cartesian", [0.4, 0.2, 0.3, 3.1416, 0.0, -1.5708], "FULL"),
#             ("joint", [-1.5708, 0.7679, -0.7679, -1.5708, 0.0, 0.0], "NONE"),
#         ]

#         self._current_motion_index = 0
#         self.get_logger().info(f"Starting motion sequence of {len(self._motions)} steps.")
#         self.send_motion_goal(*self._motions[self._current_motion_index])

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

#         self.get_logger().info(f"Sending goal {self._current_motion_index+1}: {command} -> {positions}")

#         # Wait asynchronously for the action server
#         if not self.action_client.wait_for_server(timeout_sec=5.0):
#             self.get_logger().error("Action server not available after waiting.")
#             self._motions_done_event.set()
#             return

#         future = self.action_client.send_goal_async(
#             goal_msg, feedback_callback=self.feedback_callback
#         )
#         future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error("Goal rejected by server.")
#             with self._lock:
#                 self._goal_in_progress = False
#             self._finish_or_continue()
#             return

#         self.get_logger().info("Goal accepted, waiting for result...")
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.result_callback)

#     def feedback_callback(self, feedback_msg):
#         self.get_logger().info(f"Feedback: {feedback_msg.feedback.status}")

#     def result_callback(self, future):
#         result = future.result().result
#         motion_num = self._current_motion_index + 1

#         if result.success:
#             self.get_logger().info(f"✅ Motion {motion_num} succeeded.")
#         else:
#             self.get_logger().warn(f"⚠️ Motion {motion_num} failed.")

#         with self._lock:
#             self._goal_in_progress = False

#         self._finish_or_continue()

#     def _finish_or_continue(self):
#         self._current_motion_index += 1
#         if self._current_motion_index < len(self._motions):
#             next_motion = self._motions[self._current_motion_index]
#             self.get_logger().info(
#                 f"Starting next motion ({self._current_motion_index+1}/{len(self._motions)})..."
#             )
#             threading.Thread(target=self.send_motion_goal, args=next_motion, daemon=True).start()
#         else:
#             self.get_logger().info("🎯 All motions completed successfully.")
#             self._motions_done_event.set()

#     def destroy_node(self):
#         self.get_logger().info("Shutting down BrainNode...")
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = BrainNode()
#     try:
#         rclpy.spin(node)
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
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interface.srv import StartRound
from custom_interface.action import Movement

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')

        # Service
        self.srv = self.create_service(StartRound, '/start_round', self.start_round_callback)

        # Action client
        self.action_client = ActionClient(self, Movement, '/moveit_path_plan')

        # Lock for goal in progress
        self._lock = threading.Lock()
        self._goal_in_progress = False

        # Event to wait for individual motion completion
        self._motions_done_event = threading.Event()

        # Motion sequence state
        self._motions = []
        self._current_motion_index = 0

        self.get_logger().info("Brain node started and ready.")

    # --- Service callback ---
    def start_round_callback(self, request, response):
        if not request.start:
            response.accepted = False
            response.message = "Start flag is False."
            return response

        self.get_logger().info("Service request received: start round")

        # Start motion sequence in a background thread
        threading.Thread(target=self._motion_sequence_thread, daemon=True).start()
        # self._motion_sequence_thread() // This will block the action client to receive goal response

        # Immediately return response
        response.accepted = True
        response.message = "Round started. Motions running in background."
        return response

    # --- Background thread running all motions sequentially ---
    def _motion_sequence_thread(self):
        self.get_logger().info("Motion sequence thread started.")

        # Define motions
        self._motions = [
            ("cartesian", [0.4, 0.2, 0.3, 3.1416, 0.0, -1.5708], "FULL"),
            ("cartesian", [0.4, 0.2, 0.1, 3.1416, 0.0, -1.5708], "FULL"),
            ("cartesian", [0.4, 0.2, 0.3, 3.1416, 0.0, -1.5708], "FULL"),
            ("joint",     [-1.5708, 0.7679, -0.7679, -1.5708, 0.0, 0.0], "NONE"),
        ]

        for i, (cmd, pos, constr) in enumerate(self._motions):
            self._current_motion_index = i
            self.get_logger().info(f"Sending motion {i+1}: {cmd} -> {pos}")

            # Clear event before sending motion
            self._motions_done_event.clear()
            self.send_motion_goal(cmd, pos, constr)

            # Wait here in background thread (safe!)
            self._motions_done_event.wait()  # Wait until this motion finishes
            self.get_logger().info(f"Motion {i+1} finished")

        self.get_logger().info("🎯 All motions completed.")

    # --- Send one motion goal ---
    def send_motion_goal(self, command, positions, constraint):
        with self._lock:
            if self._goal_in_progress:
                self.get_logger().warn("Goal already in progress, skipping...")
                return
            self._goal_in_progress = True

        goal_msg = Movement.Goal()
        goal_msg.command = command
        goal_msg.positions = positions
        goal_msg.constraints_identifier = constraint

        # Wait for server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available after waiting.")
            # set _motions_done_event to so .wait() stop waiting
            self._motions_done_event.set() 
            return

        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    # --- Action callbacks ---
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server.")
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
        motion_num = self._current_motion_index + 1

        if getattr(result, "success", False):
            self.get_logger().info(f"✅ Motion {motion_num} succeeded.")
        else:
            self.get_logger().warn(f"⚠️ Motion {motion_num} failed.")

        with self._lock:
            self._goal_in_progress = False

        # Signal that this motion is done
        self._motions_done_event.set()

    def destroy_node(self):
        self.get_logger().info("Shutting down BrainNode...")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

