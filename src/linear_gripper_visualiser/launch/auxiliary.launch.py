import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
import xacro
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name = 'linear_gripper_visualiser'
    xacro_path = 'urdf/ur_with_gripper.xacro'
    rviz_path = 'rviz/rviz.rviz'

    xacro_file = os.path.join(get_package_share_directory(package_name), xacro_path)
    xacro_raw_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='gripper',
            executable='gripper_server',
            name='gripper_server',
            output='screen'
        ),
        Node(
            package='moveit_path_planner',
            executable='cartesian_move_demo',
            name='cartesian_move_demo',
            output='screen',
        ),
          

    ])