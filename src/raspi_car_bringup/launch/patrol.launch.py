"""Autonomous patrol: full Nav2 stack + patrol_node looping the waypoints."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('raspi_car_bringup')
    nav2_launch = os.path.join(bringup_share, 'launch', 'nav2.launch.py')
    default_map = os.path.join(bringup_share, 'maps', 'room1.yaml')
    waypoints = PathJoinSubstitution([
        EnvironmentVariable('HOME'), '.ros', 'raspi_car', 'routes', 'room1_patrol.yaml'])

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car01'),
        DeclareLaunchArgument('waypoints_file', default_value=waypoints),
        DeclareLaunchArgument('map', default_value=default_map),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'namespace': namespace,
                'map': LaunchConfiguration('map'),
            }.items(),
        ),
        # start patrol after Nav2 has had time to activate
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='raspi_car_bringup', executable='patrol_node',
                    name='patrol_node', namespace=namespace, output='screen',
                    parameters=[{
                        'waypoints_file': LaunchConfiguration('waypoints_file'),
                        'map_yaml': LaunchConfiguration('map'),
                        'loop': True,
                        'nav_namespace': namespace,
                    }],
                ),
            ],
        ),
    ])
