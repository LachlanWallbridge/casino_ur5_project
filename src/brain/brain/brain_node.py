# ros2 service call /start_round custom_interface/srv/StartRound "{start: true}"


#!/usr/bin/env python3
import math
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from custom_interface.msg import DiceResults, CupResult
from custom_interface.srv import StartRound, GripperCmd
from custom_interface.action import Movement
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import JointState

TOOL_OFFSET = 0.13  # meters


class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        # -------------------------------------------------------------
        # REENTRANT GROUP + MULTI-THREADING
        # -------------------------------------------------------------
        self.cb_group = ReentrantCallbackGroup()

        # Action client
        self.action_client = ActionClient(
            self,
            Movement,
            '/moveit_path_plan',
            callback_group=self.cb_group
        )

        # Threading events
        self._motion_done_event = threading.Event()
        self._round_done_event = threading.Event()

        # Latest sensor states
        self.latest_dice = []
        self.latest_cup = None
        self._current_joints = None

        # Subscriptions
        self.create_subscription(
            DiceResults, '/dice_results',
            self.dice_callback, 10,
            callback_group=self.cb_group
        )

        self.create_subscription(
            CupResult, 'cup_result',
            self.cup_callback, 10,
            callback_group=self.cb_group
        )

        self.create_subscription(
            JointState, '/joint_states',
            self.joint_callback, 10,
            callback_group=self.cb_group
        )

        # Gripper client
        self.gripper_client = self.create_client(
            GripperCmd,
            'gripper_cmd',
            callback_group=self.cb_group
        )

        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for gripper service...")
        self.get_logger().info("Connected to gripper service.")

        # Start-round service
        self.create_service(
            StartRound,
            '/start_round',
            self.start_round_callback,
            callback_group=self.cb_group
        )

        self.round_active = False
        self.get_logger().info("🧠 Brain ready.")

    # -----------------------------------------------------------------
    # CALLBACKS
    # -----------------------------------------------------------------
    def dice_callback(self, msg):
        """Filter dice by bounding box area and confidence."""

        MIN_AREA = 500    # lower bound for bbox area
        MAX_AREA = 10000   # upper bound for bbox area
        CONF_THRESH = 0.60 # minimal YOLO confidence

        filtered = []

        for d in msg.dice:
            area = d.width * d.height

            if area < MIN_AREA or area > MAX_AREA:
                continue

            if d.confidence < CONF_THRESH:
                continue

            filtered.append(d)

        self.latest_dice = filtered
        self.get_logger().info(f"Detected {len(msg.dice)} dice, kept {len(self.latest_dice)} after filtering.")


    def cup_callback(self, msg):
        self.latest_cup = msg

    def joint_callback(self, msg):
        self._current_joints = list(msg.position)

    def get_current_joints(self):
        return self._current_joints.copy() if self._current_joints else None

    # -----------------------------------------------------------------
    # START ROUND SERVICE (BLOCK UNTIL THREAD FINISHES)
    # -----------------------------------------------------------------
    def start_round_callback(self, request, response):
        if not request.start:
            response.accepted = False
            response.message = "Start flag = false."
            return response

        if self.round_active:
            response.accepted = False
            response.message = "Round already running."
            return response

        self.round_active = True
        self._round_done_event.clear()

        threading.Thread(target=self.round_thread, daemon=True).start()

        self.get_logger().info("Waiting for round to complete...")
        self._round_done_event.wait()
        self.get_logger().info("Round finished.")

        response.accepted = True
        response.message = "Round completed."
        return response

    # -----------------------------------------------------------------
    # ASYNC GRIPPER COMMAND (NO LOCKS, NO GLOBAL FLAGS)
    # -----------------------------------------------------------------
    def gripper_command(self, width):
        req = GripperCmd.Request()
        req.width = width

        done = threading.Event()
        result_holder = {}

        def cb(fut):
            try:
                result_holder["ok"] = bool(fut.result().success)
            except:
                result_holder["ok"] = False
            done.set()

        future = self.gripper_client.call_async(req)
        future.add_done_callback(cb)

        done.wait()
        time.sleep(2.25)  # mechanical settle
        self.get_logger().info(f"Gripper done.")
        return result_holder["ok"]
        

    # -----------------------------------------------------------------
    # ROUND THREAD — MAIN GAME SEQUENCE
    # -----------------------------------------------------------------
    def round_thread(self):
        try:
            self.get_logger().info("🏁 Round sequence started.")

            self.cup_ready(open_gripper=True)

            # Wait for cup detection
            t = 0.0
            while self.latest_cup is None and t < 10:
                time.sleep(0.2)
                t += 0.2

            if self.latest_cup is None:
                self.get_logger().warn("Cup not detected.")
                return

            self.cup_start_xyz = (
                self.latest_cup.pose.position.x,
                self.latest_cup.pose.position.y,
                self.latest_cup.pose.position.z
            )

            # store the cups drop pose for later use
            self.cup_drop_pose = self.latest_cup.drop_pose

            # ================================================================
            # 3) EMPTY CUP (flip) and RETURN CUP
            # ================================================================
            self.empty_cup()   # flip and return to cup_start_xyz

            self.return_home(open_gripper=True)
            time.sleep(2.0)

            # Wait for exactly 2 dice
            t = 0.0
            while len(self.latest_dice) != 2 and t < 10:
                time.sleep(0.2)
                t += 0.2

            if len(self.latest_dice) != 2:
                self.get_logger().warn("Dice count != 2.")
                return

            # Pickup + place loop
            while rclpy.ok() and len(self.latest_dice) > 0:
                dice_msg = self.latest_dice[0]
                rpy = self.pickup_dice(dice_msg)
                self.drop_into_cup(rpy)
                self.return_home(open_gripper=True)
                time.sleep(1.0)

            self.get_logger().info("🎉 Round complete!")

        finally:
            self.round_active = False
            self._round_done_event.set()

    # -----------------------------------------------------------------
    # MOTION SENDER — COMPLETELY CLEAN VERSION
    # -----------------------------------------------------------------
    def send_motion(self, command, positions, constraint):
        goal = Movement.Goal()
        goal.command = command
        goal.positions = positions
        goal.constraints_identifier = constraint

        self._motion_done_event.clear()

        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Action server unavailable.")
            self._motion_done_event.set()
            return False

        fut = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        fut.add_done_callback(self.goal_response_cb)

        self._motion_done_event.wait()
        return True

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Motion rejected.")
            self._motion_done_event.set()
            return

        r = goal_handle.get_result_async()
        r.add_done_callback(self.result_cb)

    def result_cb(self, future):
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info("✔ Motion success")
            else:
                self.get_logger().warn("✖ Motion failed")
        except Exception as e:
            self.get_logger().error(f"Result callback error: {e}")

        self._motion_done_event.set()

    def feedback_callback(self, fb):
        status = getattr(fb.feedback, "status", "<no status>")
        self.get_logger().info(f"[FB] {status}")

    # -----------------------------------------------------------------
    # MOTION HELPERS (UNCHANGED LOGIC)
    # -----------------------------------------------------------------
    def pickup_dice(self, dice_msg):
        pose = dice_msg.pose
        self.get_logger().info(f"Picking up dice {dice_msg.dice_number}")

        above_xyz = [pose.position.x, pose.position.y, pose.position.z + 0.14]
        grip_xyz  = [pose.position.x, pose.position.y, pose.position.z + 0.115]

        roll, pitch, yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

        above_pose = [*above_xyz, roll, pitch, yaw]
        grip_pose  = [*grip_xyz, roll, pitch, yaw]

        self._motion_done_event.clear()
        self.send_motion("cartesian", above_pose, "FULL")
        self._motion_done_event.wait()

        self._motion_done_event.clear()
        self.send_motion("cartesian", grip_pose, "FULL")
        self._motion_done_event.wait()

        self.gripper_command(180)
        return roll, pitch, yaw

    def return_home(self, open_gripper=False):
        home = [-1.5708, 0.7679, -0.7679, -1.5708, 0.0, 0.0]
        self._motion_done_event.clear()
        self.send_motion("joint", home, "NONE")
        self._motion_done_event.wait()
        if open_gripper:
            self.gripper_command(20)

    def cup_ready(self, open_gripper=False):
        home = [
            -88.12 * math.pi / 180.0,
            65.29 * math.pi / 180.0,
            20.54 * math.pi / 180.0,
            -38.71 * math.pi / 180.0,
            -180.0 * math.pi / 180.0,
            -32.63 * math.pi / 180.0
        ]
        self._motion_done_event.clear()
        self.send_motion("joint", home, "NONE")
        self._motion_done_event.wait()
        if open_gripper:
            self.gripper_command(20)

    def drop_into_cup(self, last_rpy):
        if self.cup_drop_pose is None:
            return
        self.get_logger().info("Dropping dice into cup.")

        roll, pitch, yaw = last_rpy
        clearance = 0.04
        target = [
            self.cup_drop_pose.position.x,
            self.cup_drop_pose.position.y,
            self.cup_drop_pose.position.x + TOOL_OFFSET + clearance,
            roll, pitch, yaw
        ]

        self._motion_done_event.clear()
        self.send_motion("cartesian", target, "FULL")
        self._motion_done_event.wait()
        self.gripper_command(20)

    def empty_cup(self):
        if self.latest_cup is None:
            self.get_logger().warn("No cup detected for emptying.")
            return

        cup = self.latest_cup.pose

        q_cup = [
            cup.orientation.x,
            cup.orientation.y,
            cup.orientation.z,
            cup.orientation.w,
        ]

        cup_roll, cup_pitch, cup_yaw = euler_from_quaternion(q_cup)

        APPROACH = 0.15
        LIFT = 0.10
        DUMP_X = 0.20

        grab_pos = np.array([
            cup.position.x,
            cup.position.y,
            cup.position.z,
        ])

        above_grab_pos = grab_pos + np.array([0.0, 0.0, APPROACH])

        above_grab_pose = [
            above_grab_pos[0], above_grab_pos[1], above_grab_pos[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        grab_pose = [
            grab_pos[0], grab_pos[1], grab_pos[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        # Move above cup
        self._motion_done_event.clear()
        self.send_motion("cartesian", above_grab_pose, "FULL+WRIST1")
        self._motion_done_event.wait()
        self.get_logger().info("Above cup position reached.")

        # Move down
        self._motion_done_event.clear()
        self.send_motion("cartesian", grab_pose, "FULL+WRIST1")
        self._motion_done_event.wait()
        self.get_logger().info("Grab position reached.")

        # Grip
        self.gripper_command(180)
        self.get_logger().info("Cup gripped.")

        # Lift
        lift_pos = grab_pos + np.array([0.0, 0.0, LIFT])
        lift_pose = [
            lift_pos[0], lift_pos[1], lift_pos[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        self._motion_done_event.clear()
        self.send_motion("cartesian", lift_pose, "FULL")
        self._motion_done_event.wait()
        self.get_logger().info("Lift position reached.")

        # Dump move
        dump_pos = lift_pos + np.array([DUMP_X, 0.0, 0.0])
        dump_pose = [
            dump_pos[0], dump_pos[1], dump_pos[2],
            cup_roll, cup_pitch, 0,
        ]

        self._motion_done_event.clear()
        self.send_motion("cartesian", dump_pose, "FULL")
        self._motion_done_event.wait()
        self.get_logger().info("Dump position reached.")

        # Rotate wrist3
        current = self.get_current_joints()
        if current is None:
            self.get_logger().error("No joint states available for flip.")
            return

        target = current.copy()
        target[4] += math.pi * 3/4
        target[4] = math.atan2(math.sin(target[4]), math.cos(target[4]))

        self._motion_done_event.clear()
        self.send_motion("joint", target, "NONE")
        self._motion_done_event.wait()
        self.get_logger().info("Wrist twist completed.")

        time.sleep(1.0)

        # Return wrist
        self._motion_done_event.clear()
        self.send_motion("joint", current, "NONE")
        self._motion_done_event.wait()
        self.get_logger().info("Wrist return completed.")

        # Return cup home
        cup_start = np.array(self.cup_start_xyz)

        above_return_pos = cup_start + np.array([0.0, 0.0, LIFT])
        above_return_pose = [
            above_return_pos[0], above_return_pos[1], above_return_pos[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        self._motion_done_event.clear()
        self.send_motion("cartesian", above_return_pose, "FULL")
        self._motion_done_event.wait()
        self.get_logger().info("Above return position reached.")

        place_pose = [
            cup_start[0], cup_start[1], cup_start[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        self._motion_done_event.clear()
        self.send_motion("cartesian", place_pose, "FULL")
        self._motion_done_event.wait()
        self.get_logger().info("Place position reached.")

        self.gripper_command(20)
        self.get_logger().info("Cup released.")

        self._motion_done_event.clear()
        self.send_motion("cartesian", above_return_pose, "FULL")
        self._motion_done_event.wait()
        self.get_logger().info("Above return position reached.")


# -----------------------------------------------------------------
# MAIN
# -----------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Brain()

    # ⭐ RECOMMENDED EXECUTOR SIZE ⭐
    executor = MultiThreadedExecutor(num_threads=7)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
