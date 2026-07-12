"""Start Nav2 plus a service-based field waypoint recorder."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('raspi_car_bringup')
    nav2_launch = os.path.join(share, 'launch', 'nav2.launch.py')
    default_map = os.path.join(share, 'maps', 'room1.yaml')
    default_route = PathJoinSubstitution([
        EnvironmentVariable('HOME'), '.ros', 'raspi_car', 'routes', 'room1_patrol.yaml'])
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car01'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('output_file', default_value=default_route),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={'namespace': namespace, 'map': LaunchConfiguration('map')}.items(),
        ),
        Node(
            package='raspi_car_bringup', executable='waypoint_recorder_node',
            name='waypoint_recorder', namespace=namespace, output='screen',
            parameters=[{
                'output_file': LaunchConfiguration('output_file'),
                'map_yaml': LaunchConfiguration('map'),
            }],
        ),
    ])
