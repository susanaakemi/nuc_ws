#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_pkg',
            executable='arm.launch',
            name='arm.launch'
        ),
        Node(
            package='camera_pkg',
            executable='camera.launch',
            name='camera.launch'
        ),
        Node(
            package='odometry_pkg',
            executable='odometry.launch',
            name='odometry.launch'
        ),
      
