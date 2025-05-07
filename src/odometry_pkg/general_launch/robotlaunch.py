from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_pkg',
            executable='clicked_point_subscriber',
            name='clicked_point_subscriber'
        ),
        Node(
            package='camera_pkg',
            executable='framepublisher',
            name='framepublisher'
        ),
        Node(
            package='camera_pkg',
            executable='pixel_to_point3d',
            name='pixel_to_point3d'
        ),
        Node(
            package='odometry_pkg',
            executable='controller',
            name='controller'
        ),
        Node(
            package='odometry_pkg',
            executable='efk',
            name='ExtendedKalmanFilter'
        ),
        Node(
            package='odometry_pkg',
            executable='odometry_node',
            name='odometry'
        ),
        Node(
            package='odometry_pkg',
            executable='utils',
            name='utils'
        ),
    ])
