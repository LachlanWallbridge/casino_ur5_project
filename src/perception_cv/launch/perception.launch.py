#!/usr/bin/env python3
import os
 
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_cv',
            executable='aruco_cv',
            name='aruco_cv',
            output='screen'
        ),
        Node(
            package='perception_cv',
            executable='dice_cv',
            name='dice_cv',
            output='screen'
        ),
        Node(
            package='perception_cv',
            executable='player_cv',
            name='player_cv',
            output='screen'
        ),
        # Node(
        #     package='perception_cv',
        #     executable='cup_cv',
        #     name='cup_cv',
        #     output='screen'
        # ),
        
    ])