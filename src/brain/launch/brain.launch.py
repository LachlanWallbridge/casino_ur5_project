#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frontend_arg = DeclareLaunchArgument(
        'frontend',
        default_value='true',
        description='Whether to launch the frontend and backend (true/false)'
    )

    frontend = LaunchConfiguration('frontend')

    def launch_setup(context, *args, **kwargs):
        frontend_enabled = context.perform_substitution(frontend).lower() == 'true'

        # -----------------------------------
        # Brain node
        # -----------------------------------
        brain_node = Node(
            package='brain',
            executable='brain_node',
            name='brain',
            output='screen'
        )

        # -----------------------------------
        # rosbridge (execute like CLI)
        # -----------------------------------
        rosbridge_proc = ExecuteProcess(
            cmd=[
                "ros2", "launch", "rosbridge_server",
                "rosbridge_websocket_launch.xml"
            ],
            output="screen"
        )

        # # -----------------------------------
        # # Optional frontend/backend terminals
        # # -----------------------------------
        # optional_processes = []
        # if frontend_enabled:
        #     backend = ExecuteProcess(
        #         cmd=['bash', '-c', 'cd ~/path/to/backend && uvicorn api:app --reload --port 8000'],
        #         output='screen'
        #     )
 
        #     frontend_proc = ExecuteProcess(
        #         cmd=['bash', '-c', 'cd ~/path/to/frontend && npm start'],
        #         output='screen'
        #     )
 
        #     optional_processes.extend([backend, frontend_proc])
 
        return [brain_node, rosbridge_proc] # + optional_processes
    return LaunchDescription([
        frontend_arg,
        OpaqueFunction(function=launch_setup)
    ])
