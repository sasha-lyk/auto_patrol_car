import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('raspi_car_sensors')
    default_config = os.path.join(share, 'config', 'yb_mra02_uart.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car01'),
        DeclareLaunchArgument('imu_config', default_value=default_config),
        Node(
            package='raspi_car_sensors', executable='yb_mra02_uart_node',
            name='yb_mra02_uart_node',
            namespace=LaunchConfiguration('namespace'), output='screen',
            parameters=[LaunchConfiguration('imu_config')],
        ),
    ])
