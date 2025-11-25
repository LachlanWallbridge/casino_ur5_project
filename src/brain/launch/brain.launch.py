#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
 
 
def generate_launch_description():
    # --- Launch argument: frontend ---
    frontend_arg = DeclareLaunchArgument(
        'frontend',
        default_value='true',
        description='Whether to launch the frontend and backend (true/false)'
    )
 
    frontend = LaunchConfiguration('frontend')
 
    def launch_setup(context, *args, **kwargs):
        # --- Get resolved frontend flag (bool) ---
        frontend_enabled = context.perform_substitution(frontend).lower() == 'true'
        manual_start_value = not frontend_enabled
 
        # --- Brain Node ---
        brain_node = Node(
            package='brain', 
            executable='brain_node',      
            name='brain',
            output='screen',
            parameters=[{'manual_start': manual_start_value}]
        )
 
        # --- rosbridge (WebSocket) ---
        rosbridge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('rosbridge_server'),
                    'launch',
                    'rosbridge_websocket_launch.xml'
                )
            ])
        )
 
        # --- Optional frontend/backend processes ---
        optional_processes = []
        if frontend_enabled:
            backend = ExecuteProcess(
                cmd=['bash', '-c', 'cd ~/path/to/backend && uvicorn api:app --reload --port 8000'],
                output='screen'
            )
 
            frontend_proc = ExecuteProcess(
                cmd=['bash', '-c', 'cd ~/path/to/frontend && npm start'],
                output='screen'
            )
 
            optional_processes.extend([backend, frontend_proc])
 
        return [brain_node, rosbridge_launch] + optional_processes
 
    return LaunchDescription([
        frontend_arg,
        OpaqueFunction(function=launch_setup)
    ])