"""Mapping bringup: base(+EKF) + lidar + slam_toolbox.

Run this, teleop/web-drive the car around the room, then save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/room1 --ros-args -r __ns:=/car01
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('raspi_car_bringup')
    base_mux_launch = os.path.join(bringup_share, 'launch', 'base_with_mux.launch.py')
    lidar_launch = os.path.join(bringup_share, 'launch', 'rplidar_a1.launch.py')
    slam_config = os.path.join(bringup_share, 'config', 'slam_toolbox_mapping.yaml')

    namespace = LaunchConfiguration('namespace')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car01'),
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_mux_launch),
            launch_arguments={'namespace': namespace}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            condition=IfCondition(LaunchConfiguration('start_lidar')),
            launch_arguments={
                'namespace': namespace,
                'serial_port': LaunchConfiguration('lidar_port'),
            }.items(),
        ),
        # start SLAM after base+lidar TFs are up
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='slam_toolbox', executable='async_slam_toolbox_node',
                    name='slam_toolbox', namespace=namespace, output='screen',
                    parameters=[slam_config],
                    remappings=[('scan', scan_topic)],
                ),
            ],
        ),
    ])
