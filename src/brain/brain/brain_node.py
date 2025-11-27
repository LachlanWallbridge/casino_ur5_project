#!/usr/bin/env python3
import math
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interface.msg import DiceResults, CupResult
from custom_interface.srv import StartRound, GripperCmd
from custom_interface.action import Movement
from tf_transformations import euler_from_quaternion
import transforms3d.quaternions as tq
from sensor_msgs.msg import JointState

TOOL_OFFSET = 0.13  # meters

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

        # ---- Cup subscriber ----
        self.latest_cup = None
        self.create_subscription(CupResult, 'cup_result', self.cup_callback, 10)

        # Joint state holder
        self._current_joints = None
        self.create_subscription(JointState, "/joint_states", self._joint_states_cb, 10)

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

    # ================================================================
    #   CALLBACKS
    # ================================================================
    def dice_callback(self, msg):
        self.latest_dice = msg.dice

    def cup_callback(self, msg: CupResult):
        self.latest_cup = msg

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
        
        time.sleep(3.0)
        
        if res.success:
            self.get_logger().info(f"Gripper set to width {width}.")
        else:
            self.get_logger().warn(f"Gripper command failed: {res.message}")

        return res.success


    def _joint_states_cb(self, msg: JointState):
        # msg.position is a tuple of 6 values (UR5e)
        self._current_joints = list(msg.position)

    def get_current_joints(self):
        if self._current_joints is None:
            self.get_logger().warn("Joint states not received yet.")
            return None
        return self._current_joints.copy()
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
        self.get_logger().info("🏁 Starting round sequence...")

        self.cup_ready(open_gripper=True)

        # ================================================================
        # 1) WAIT FOR CUP DETECTION
        # ================================================================
        self.get_logger().info("Waiting for cup to be detected...")
        t = 0.0
        while self.latest_cup is None and t < 10.0:
            time.sleep(0.2)
            t += 0.2

        if self.latest_cup is None:
            self.get_logger().warn("Cup not detected — aborting round.")
            self.round_active = False
            return

        # ================================================================
        # 2) PICK UP CUP
        # ================================================================
        # store the cup's precise initial world position 
        self.cup_start_xyz = (
            self.latest_cup.pose.position.x,
            self.latest_cup.pose.position.y,
            self.latest_cup.pose.position.z
        )

        # ================================================================
        # 3) EMPTY CUP (flip) and RETURN CUP
        # ================================================================
        self.empty_cup()   # flip and return to cup_start_xyz

        # ================================================================
        # 4) MOVE HOME AFTER CUP RESET
        # ================================================================
        self.return_home(open_gripper=True)

        # Give cv 2 seconds to re-detect dice without cup shadow
        time.sleep(2.0)

        # ================================================================
        # 5) NOW HANDLE DICE PICKUP / DROP INTO CUP
        # ================================================================
        self.get_logger().info("Waiting for dice...")
        t = 0.0
        while len(self.latest_dice) != 2 and t < 10.0:
            time.sleep(0.2)
            t += 0.2

        if len(self.latest_dice) != 2:
            self.get_logger().warn(f"Only {len(self.latest_dice)} dice detected. Ending round.")
            self.round_active = False
            return

        # Sequence: pickup dice -> drop into cup -> home
        while rclpy.ok() and len(self.latest_dice) > 0:
            dice_msg = self.latest_dice[0]

            # pickup returns roll/pitch/yaw
            last_rpy = self.pickup_dice(dice_msg)

            # DROP DICE INTO CUP (NEW: uses dice RPY)
            self.drop_into_cup(last_rpy)

            # HOME
            self.return_home(open_gripper=True)

            time.sleep(1.0)

        self.get_logger().info("🎉 Round complete.")
        self.round_active = False



    # ================================================================
    #   DICE PICKUP
    # ================================================================
    def pickup_dice(self, dice_msg):
        """Pick up a single dice. Returns (roll, pitch, yaw) for later use."""
        
        self.get_logger().info(f"Picking up dice with value {dice_msg.dice_number}")
        pose = dice_msg.pose

        # ---- Build EE poses ----
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

        # Extract RPY from dice orientation
        roll, pitch, yaw = euler_from_quaternion(above[3:])
        above_pose = [above[0], above[1], above[2], roll, pitch, yaw]
        dice_pose  = [dice[0],  dice[1],  dice[2],  roll, pitch, yaw]

        # ---- Move above dice ----
        self._motion_done_event.clear()
        self.send_motion("cartesian", above_pose, "FULL")
        self._motion_done_event.wait()
        self.get_logger().info("Reached above dice.")

        # ---- Move down onto dice ----
        self._motion_done_event.clear()
        self.send_motion("cartesian", dice_pose, "FULL")
        self._motion_done_event.wait()
        self.get_logger().info("Reached dice position.")

        # ---- Grip dice ----
        if not self.gripper_command(180):
            self.get_logger().warn("Gripper failed to close on dice.")
        else:
            self.get_logger().info("Dice gripped.")

        # Return the dice orientation for later use in dropping into cup
        return roll, pitch, yaw


    def return_home(self, open_gripper: bool = False):
        """Return the robot to the standard home joint position."""
        
        home_joints = [-1.5708, 0.7679, -0.7679, -1.5708, 0.0, 0.0]

        self.get_logger().info("Returning to home position.")

        self._motion_done_event.clear()
        self.send_motion("joint", home_joints, "NONE")
        self._motion_done_event.wait()

        self.get_logger().info("Home motion finished.")

        if open_gripper:
            if not self.gripper_command(20):
                self.get_logger().warn("Failed to open gripper at home.")
            else:
                self.get_logger().info("Gripper opened at home.")

    
    def cup_ready(self, open_gripper: bool = False):
        """Return the robot to the standard home joint position."""
        
        # -88.12, 65.29, 20.54, 38.71, 0, -32.63, 
        home_joints = [float(-88.12 * math.pi / 180.0), float(65.29 * math.pi / 180.0), float(20.54 * math.pi / 180.0), float(-38.71 * math.pi / 180.0), float(-180 * math.pi / 180.0), float(-32.63 * math.pi / 180.0)]

        self.get_logger().info("Returning to home position.")

        self._motion_done_event.clear()
        self.send_motion("joint", home_joints, "NONE")
        self._motion_done_event.wait()

        self.get_logger().info("Home motion finished.")

        if open_gripper:
            if not self.gripper_command(20):
                self.get_logger().warn("Failed to open gripper at home.")
            else:
                self.get_logger().info("Gripper opened at home.")

    def drop_into_cup(self, last_rpy):
        if self.cup_start_xyz is None:
            return
            

        self.get_logger().info("Dropping dice into cup.")

        roll, pitch, yaw = last_rpy

        clearance = 0.04

        target = [
            self.cup_start_xyz[0],
            self.cup_start_xyz[1],
            self.cup_start_xyz[2] + TOOL_OFFSET + clearance,
            roll,
            pitch,
            yaw
        ]

        self._motion_done_event.clear()
        self.send_motion("cartesian", target, "FULL")
        self._motion_done_event.wait()

        # Open gripper (release dice)
        self.gripper_command(20)


    def empty_cup(self):
        """Pick up the cup, move it, flip 180° around its own Z axis to empty it,
        then return it to the start location with safe approach/retreat poses.
        """

        if self.latest_cup is None:
            self.get_logger().warn("No cup detected for emptying.")
            return

        self.get_logger().info("Picking up and emptying cup...")

        # ------------------------------------------------------------
        # Extract cup pose + orientation
        # ------------------------------------------------------------
        cup = self.latest_cup.pose

        q_cup = [
            cup.orientation.x,
            cup.orientation.y,
            cup.orientation.z,
            cup.orientation.w,
        ]


        # Cup RPY
        cup_roll, cup_pitch, cup_yaw = euler_from_quaternion(q_cup)

        # Tunable heights
        APPROACH = 0.15   # approach height above grab / place
        LIFT     = 0.10   # lift height for moving around
        DUMP_X   = 0.20   # shift along global X to dump

        # ------------------------------------------------------------
        # Compute grab position along cup's Z axis
        # ------------------------------------------------------------
        grab_pos = np.array([
            cup.position.x,
            cup.position.y,
            cup.position.z,
           ]) 
        # ]) + TOOL_OFFSET * z_axis

        # Above-grab pose (like dice "above")
        above_grab_pos = grab_pos + np.array([0.0, 0.0, APPROACH])

        above_grab_pose = [
            above_grab_pos[0], above_grab_pos[1], above_grab_pos[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        grab_pose = [
            grab_pos[0], grab_pos[1], grab_pos[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        # ------------------------------------------------------------
        # 1. Approach from above, then move down and grip
        # ------------------------------------------------------------
        # Move above cup
        self._motion_done_event.clear()
        self.send_motion("cartesian", above_grab_pose, "FULL+WRIST1")
        self._motion_done_event.wait()

        # Move down to grab
        self._motion_done_event.clear()
        self.send_motion("cartesian", grab_pose, "FULL+WRIST1")
        self._motion_done_event.wait()

        # Close gripper
        self.gripper_command(180)

        # ------------------------------------------------------------
        # 2. Lift up and move to dump location
        # ------------------------------------------------------------
        lift_pos = grab_pos + np.array([0.0, 0.0, LIFT])
        lift_pose = [
            lift_pos[0], lift_pos[1], lift_pos[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        self._motion_done_event.clear()
        self.send_motion("cartesian", lift_pose, "FULL")
        self._motion_done_event.wait()

        dump_pos = lift_pos + np.array([DUMP_X, 0.0, 0.0])
        dump_pose = [
            dump_pos[0], dump_pos[1], dump_pos[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        self._motion_done_event.clear()
        self.send_motion("cartesian", dump_pose, "FULL")
        self._motion_done_event.wait()

        self.get_logger().info("Reached dump location.")

        # ------------------------------------------------------------
        # 3. Flip cup 180° by rotating wrist3 +π
        # ------------------------------------------------------------
        
        self.get_logger().info("Flipping cup by rotating wrist3 180°.")
        current = self.get_current_joints()
        if current is None:
            self.get_logger().error("No joint states available for flip.")
            return

        # copy
        target = current.copy()

        # add +π to wrist3 (joint index 5)
        target[4] += math.pi * 3/4

        # normalize angle to [-π, +π]
        target[4] = math.atan2(math.sin(target[4]), math.cos(target[4]))

        # send joint motion
        self._motion_done_event.clear()
        self.send_motion("joint", target, "NONE")
        self._motion_done_event.wait()

        self.get_logger().info("Wrist3 rotated 180° to flip cup.")

        time.sleep(1.0)  # allow dice to fall

        # ------------------------------------------------------------
        # 4. Return wrist3 to original angle
        # ------------------------------------------------------------
        self._motion_done_event.clear()
        self.send_motion("joint", current, "NONE")
        self._motion_done_event.wait()

        # ------------------------------------------------------------
        # 5. Return cup to start location with above/below pattern
        # ------------------------------------------------------------
        cup_start = np.array(self.cup_start_xyz)

        # Above the return location
        above_return_pos = cup_start + np.array([0.0, 0.0, LIFT])
        above_return_pose = [
            above_return_pos[0], above_return_pos[1], above_return_pos[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        # Move from dump back to above return
        self._motion_done_event.clear()
        self.send_motion("cartesian", above_return_pose, "FULL")
        self._motion_done_event.wait()

        # Descend to final placement pose
        place_pose = [
            cup_start[0], cup_start[1], cup_start[2],
            cup_roll, cup_pitch, cup_yaw,
        ]

        self._motion_done_event.clear()
        self.send_motion("cartesian", place_pose, "FULL")
        self._motion_done_event.wait()

        # Open gripper to release cup
        self.gripper_command(20)

        # Retreat back up so home move is clear of the cup
        self._motion_done_event.clear()
        self.send_motion("cartesian", above_return_pose, "FULL")
        self._motion_done_event.wait()

        self.get_logger().info("Cup emptied and returned, EE retreated above cup.")




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
