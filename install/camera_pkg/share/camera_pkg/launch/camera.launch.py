#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pkg',
            executable='framepublisher',
            name='framepublisher'
        ),
        Node(
            package='camera_pkg',
            executable='pixel_to_point3d',
            name='pixel_to_point3d'
        )
    ])
