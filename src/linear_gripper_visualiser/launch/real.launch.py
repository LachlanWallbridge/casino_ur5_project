import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Toggle between simulated or real UR5e hardware
use_fake = False
use_fake_str = 'true'
ur_type = 'ur5e'
ip_address = 'yyy.yyy.yyy.yyy'

if not use_fake:
    print("not fake")
    ip_address = '192.168.0.100'
    use_fake_str = 'false'


def get_realsense_launch():
    """Setup Realsense camera launch with complete pointcloud configuration."""
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'true',
            'color_width': '1920',
            'color_height': '1080',
            'color_fps': '6','depth_height': '480',
            
            'depth_width': '848',
            'depth_height': '480',
            'depth_fps': '6',
            'pointcloud_texture_stream': 'RS2_STREAM_COLOR',
            'pointcloud_texture_index': '0',
            'filters': 'pointcloud',
            'allow_no_texture_points': 'false'
        }.items()
    )

def get_ur_control_launch():
    """Configure UR control launch for the UR5e arm."""
    end_effector_path = os.path.join(
        get_package_share_directory('linear_gripper_visualiser'), 'urdf', 'ur_with_gripper.urdf.xacro'
    )


    ur_control_launch_args = {
        'ur_type': ur_type,
        'robot_ip': ip_address,
        'use_fake_hardware': use_fake_str,
        'launch_rviz': 'false',  
        'description_file': end_effector_path,
    }

    # Add controller if using simulated hardware
    if use_fake:
        ur_control_launch_args['initial_joint_controller'] = 'joint_trajectory_controller'

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])
        ),
        launch_arguments=ur_control_launch_args.items(),
    )

def get_moveit_launch():
    """Configure MoveIt launch with a delay to ensure UR control is initialized."""
    moveit_launch_args = {
        'ur_type': ur_type, #Broski is this real??
        'robot_ip': ip_address,
        'launch_rviz': 'true',
        'use_fake_hardware': use_fake_str,
    }
            

    return TimerAction(
        period=4.0,  # Delay to prevent conflicts in RViz
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py'])
                ),
                launch_arguments=moveit_launch_args.items(),
            )
        ]
    )


def get_auxiliary_launch():
    moveit_launch_path = os.path.join(
        get_package_share_directory('linear_gripper_visualiser'), 'launch', 'auxiliary.launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_path)
    )



def generate_launch_description():
    """Main function to generate the complete launch description."""
    launch_description = [
        get_ur_control_launch(),
        get_moveit_launch(),
        # get_rviz_launch(),
    ]

    # Only add camera launch if using real hardware
    #if not use_fake:
        #launch_description.append(get_realsense_launch())
        #launch_description.append(get_auxiliary_launch())

    return LaunchDescription(launch_description)