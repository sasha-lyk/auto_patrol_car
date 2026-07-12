import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('raspi_car_bringup')
    default_config = os.path.join(bringup_share, 'config', 'rplidar_a1.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car01'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_config', default_value=default_config),
        Node(
            package='rplidar_ros', executable='rplidar_composition',
            name='rplidar_node', namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('lidar_config'),
                {'serial_port': LaunchConfiguration('serial_port')},
            ],
            remappings=[('scan', 'scan')],
        ),
    ])
