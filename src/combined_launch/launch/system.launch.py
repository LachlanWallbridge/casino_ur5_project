#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --------------------------------------------------
    # 1. Ur, Moveit, move action, gripper, etc (real.launch.py) (must start first)
    # --------------------------------------------------
    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('linear_gripper_visualiser'),
                'launch',
                'real.launch.py'
            )
        )
    )

    # --------------------------------------------------
    # 2. Perception
    # --------------------------------------------------
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('perception_cv'),
                'launch',
                'perception.launch.py'
            )
        )
    )

    # --------------------------------------------------
    # 3. Brain
    # --------------------------------------------------
    brain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('brain'),
                'launch',
                'brain.launch.py'
            )
        )
    )

    # --------------------------------------------------
    # EVENT HANDLERS
    # --------------------------------------------------

    # When gripper starts, launch perception
    start_perception_after_gripper = RegisterEventHandler(
        OnProcessStart(
            target_action=gripper,
            on_start=[perception]
        )
    )

    # When perception starts, launch brain
    start_brain_after_perception = RegisterEventHandler(
        OnProcessStart(
            target_action=perception,
            on_start=[brain]
        )
    )

    return LaunchDescription([
        gripper,
        start_perception_after_gripper,
        start_brain_after_perception
    ])
