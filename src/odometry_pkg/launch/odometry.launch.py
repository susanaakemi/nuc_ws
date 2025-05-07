#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odometry_pkg',
            executable='controller',
            name='controller'
        ),
        Node(
            package='odometry_pkg',
            executable='efk',
            name='efk'
        ),
        Node(
            package='odometry_pkg',
            executable='odometry_node',
            name='odometry_node'
        ),
        Node(
            package='odometry_pkg',
            executable='utils',
            name='utils'
        )
    ])
