from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction

import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    package_name = 'linear_gripper_visualiser'
    xacro_path = 'urdf/ur_with_gripper.urdf.xacro'
    rviz_path = 'rviz/rviz.rviz'
    urdf_path = 'urdf/linear_gripper.urdf'

    xacro_file = os.path.join(get_package_share_directory(package_name), xacro_path)
    xacro_raw_description = xacro.process_file(xacro_file).toxml()
    rviz_file = os.path.join(get_package_share_directory(package_name), rviz_path)

    return LaunchDescription([
        # Declare argument for custom URDF
        DeclareLaunchArgument(
            'urdf_path',
            default_value=urdf_path,
            description='Path to custom URDF'
        ),

        # Driver server launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ur_robot_driver'),
                    'launch',
                    'ur_control.launch.py'
                )
            ]),
            launch_arguments={
                'ur_type': 'ur5e',
                'robot_ip': '192.168.0.100',
                'use_fake_hardware': 'false',
                'launch_rviz': 'false',
                'description_file': os.path.join(get_package_share_directory('linear_gripper_visualiser'), 'urdf/ur_with_gripper.urdf.xacro')
            }.items()
        ),

        # Moveit server launch (after a delay)
        # If you need a delay, use TimerAction
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        os.path.join(
                            get_package_share_directory('ur_moveit_config'),
                            'launch',
                            'ur_moveit.launch.py'
                        )
                    ]),
                    launch_arguments={
                        'ur_type': 'ur5e',
                        'launch_rviz': 'true',
                    }.items()
                )
            ]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': xacro_raw_description}]
        ),
        Node(
            package = 'joint_state_publisher',
            executable = 'joint_state_publisher',
            name = 'joint_state_publisher',
            output = 'screen',
            parameters=[{'robot_description': xacro_raw_description}]       
        ),
    ])