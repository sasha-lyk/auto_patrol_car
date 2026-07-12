import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('raspi_car_base')
    default_config = os.path.join(package_share, 'config', 'l298n.yaml')

    namespace = LaunchConfiguration('namespace')
    base_config = LaunchConfiguration('base_config')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car01'),
        DeclareLaunchArgument('base_config', default_value=default_config),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='cmd_vel',
            description='Input Twist for the motor node (mux output)'),

        # 1) quadrature encoder reader -> wheel_ticks_std / wheel_speeds_std
        Node(
            package='raspi_car_base', executable='encoder_node',
            name='encoder_node', namespace=namespace, output='screen',
            parameters=[base_config],
        ),
        # 2) closed-loop wheel odometry (measured motion) -> odom
        Node(
            package='raspi_car_base', executable='wheel_odometry_node',
            name='wheel_odometry_node', namespace=namespace, output='screen',
            parameters=[base_config],
        ),
        # 3) L298N motor node with PID velocity control
        Node(
            package='raspi_car_base', executable='l298n_motor_node',
            name='l298n_motor_node', namespace=namespace, output='screen',
            parameters=[base_config, {'cmd_vel_topic': cmd_vel_topic}],
        ),
    ])
