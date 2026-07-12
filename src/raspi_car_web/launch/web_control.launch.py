"""Web control bringup: turn_assist is already in base; this starts the Flask
ROS2 bridge that serves the dashboard and republishes /api/cmd -> cmd_vel_web.

Run AFTER base_with_mux (or nav2/patrol) so the mux + motor nodes exist.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    http_port = LaunchConfiguration('http_port')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car01'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        SetEnvironmentVariable('CAR_NAMESPACE', namespace),
        SetEnvironmentVariable('HTTP_PORT', http_port),
        Node(
            package='raspi_car_web', executable='backend_ros2',
            name='web_cmd_bridge', namespace=namespace, output='screen',
        ),
    ])
