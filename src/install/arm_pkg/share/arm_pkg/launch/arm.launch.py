#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_pkg',
            executable='clicked_point_subscriber',
            name='clicked_point_subscriber'
        )
    ])
