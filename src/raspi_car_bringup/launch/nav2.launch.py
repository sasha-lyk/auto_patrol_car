"""Navigation bringup: base(+EKF) + lidar + Nav2 stack.

Unlike the old version, this wires the CUSTOM nav2_car01.yaml and the saved
room1 map by DEFAULT, so `ros2 launch raspi_car_bringup nav2.launch.py` just
works without extra args.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('raspi_car_bringup')
    base_mux_launch = os.path.join(bringup_share, 'launch', 'base_with_mux.launch.py')
    lidar_launch = os.path.join(bringup_share, 'launch', 'rplidar_a1.launch.py')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')

    default_params = os.path.join(bringup_share, 'config', 'nav2_car01.yaml')
    default_map = os.path.join(bringup_share, 'maps', 'room1.yaml')

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car01'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': 'True',
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'False',
            }.items(),
        ),
    ])
