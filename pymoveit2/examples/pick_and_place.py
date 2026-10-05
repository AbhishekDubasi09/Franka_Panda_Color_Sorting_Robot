#!/usr/bin/env python3
"""
Pick and place node combining Cartesian and joint-space moves with smooth joint transitions.
Locks the detected color coordinates before starting the motion.

ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B

Sort several colors in one run, in the given order:
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_colors:=RGB

"""

from threading import Thread
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

import math


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        self.target_color = self.get_parameter("target_color").value.upper()

        # Optional ordered sequence, e.g. "RGB". Overrides target_color.
        self.declare_parameter("target_colors", "")
        seq = str(self.get_parameter("target_colors").value).upper()
        self.sequence = [c for c in seq if c in "RGB"] or [self.target_color]
        self.latest = {}  # color -> (coords, receive time)

        # Detector z is the true object-top height in the base frame; hover
        # this far above it (matches the original 1.1 - 0.60 = 0.5 m hover).
        # Set legacy_coords:=true to drive the original detector, whose z
        # is offset by a fixed 0.60.
        self.declare_parameter("hover_above_top", 0.384)
        self.declare_parameter("legacy_coords", False)
        self.hover_above_top = float(self.get_parameter("hover_above_top").value)
        self.legacy_coords = bool(self.get_parameter("legacy_coords").value)

        self.declare_parameter("approach_offset", 0.31)
        self.approach_offset = float(
            self.get_parameter("approach_offset").value
        )
        # Flags
        self.already_moved = False
        self.target_coords = None  # Stores the locked coordinates

        self.callback_group = ReentrantCallbackGroup()

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set lower velocity & acceleration for smoother motion
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Subscriber
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )
        self.get_logger().info(
            f"Waiting for {'/'.join(self.sequence)} from /color_coordinates...")

        # Predefined joint positions (in radians)
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
                             math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)]

        # Move to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

    def coords_callback(self, msg):
        try:
            color_id, x, y, z = msg.data.split(",")
            self.latest[color_id.strip().upper()] = (
                [float(x), float(y), float(z)], time.monotonic())
        except Exception as e:
            self.get_logger().error(f"Error parsing /color_coordinates: {e}")

    def wait_for_target(self, color, max_age=1.0, timeout=60.0):
        """Return fresh coordinates for `color`, or None on timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            entry = self.latest.get(color)
            if entry and time.monotonic() - entry[1] < max_age:
                return entry[0]
            time.sleep(0.1)
        return None

    def run_sequence(self):
        for color in self.sequence:
            coords = self.wait_for_target(color)
            if coords is None:
                self.get_logger().warn(f"{color} not seen, skipping.")
                continue
            self.target_color = color
            self.target_coords = coords
            self.get_logger().info(
                f"Target {color} locked at: "
                f"[{coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f}]")
            self.pick_and_place(coords)
            self.latest.pop(color, None)
        self.get_logger().info("Pick-and-place sequence complete.")
        rclpy.shutdown()

    def pick_and_place(self, coords):
        hover_z = (coords[2] - 0.60 if self.legacy_coords
                   else coords[2] + self.hover_above_top)
        pick_position = [coords[0], coords[1], hover_z]
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]

        # 1. Move to home joint configuration
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 2. Move above target (Cartesian)
        self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 3. Open gripper
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 4. Move down to approach object
        approach_position = [
            pick_position[0],
            pick_position[1],
            pick_position[2] - self.approach_offset
        ]

        self.moveit2.move_to_pose(
            position=approach_position,
            quat_xyzw=quat_xyzw,
            cartesian=True
        )
        self.moveit2.wait_until_executed()

        # 5. Close gripper
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 6. Lift up back to pick_position
        # self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
        # self.moveit2.wait_until_executed()

        # 7. Move to home joint configuration
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 8. Move to drop joint configuration
        self.moveit2.move_to_configuration(self.drop_joints)
        self.moveit2.wait_until_executed()

        # 9. Open gripper to release
        self.gripper.open()
        self.gripper.wait_until_executed()

        # 10. Close gripper
        self.gripper.close()
        self.gripper.wait_until_executed()

        # 11. Return to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.run_sequence()  # executor thread is a daemon; exit when done
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
