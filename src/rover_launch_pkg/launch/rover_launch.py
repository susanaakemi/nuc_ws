from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    ros2_setup = '/home/robotec/ros2_ws/install'

    return LaunchDescription([
        # LIDAR
        ExecuteProcess(
            cmd=['ros2', 'launch', 'sllidar_ros2', 'sllidar_a1_launch.py'],
            shell=True,
            output='screen',
            additional_env={'AMENT_PREFIX_PATH': ros2_setup}
        ),

        # Cámara Astra
        ExecuteProcess(
            cmd=['ros2', 'launch', 'astra_camera', 'astro_pro_plus.launch.xml'],
            shell=True,
            output='screen',
            additional_env={'AMENT_PREFIX_PATH': ros2_setup}
        ),

        # Nodo framepublisher
        ExecuteProcess(
            cmd=['ros2', 'run', 'camera_pkg', 'framepublisher'],
            shell=True,
            output='screen',
            additional_env={'AMENT_PREFIX_PATH': ros2_setup}
        ),

        # Nodo pixel_to_point3d
        ExecuteProcess(
            cmd=['ros2', 'run', 'camera_pkg', 'pixel_to_point3d'],
            shell=True,
            output='screen',
            additional_env={'AMENT_PREFIX_PATH': ros2_setup}
        ),

        # Nodo de odometría
        ExecuteProcess(
            cmd=['ros2', 'run', 'odometry_pkg', 'odometry_node'],
            shell=True,
            output='screen',
            additional_env={'AMENT_PREFIX_PATH': ros2_setup}
        ),
    ])
