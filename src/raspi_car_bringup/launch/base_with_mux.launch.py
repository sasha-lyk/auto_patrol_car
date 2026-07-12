"""Base bringup: encoders + closed-loop motor + EKF + mux + turn-assist.

Node graph (namespace car01):

  cmd_vel_web ─> turn_assist ─> cmd_vel_web_assisted ─┐
  cmd_vel_teleop ────────────────────────────────────┤
  cmd_vel_nav (from Nav2 collision_monitor) ──────────┤─> cmd_vel_mux ─> cmd_vel
  emergency_stop(True) ─> latched inhibit ────────────────────────────────┤
                                                                            v
                                    encoder_node ─> wheel_speeds_std ─> l298n_motor_node (PID)
                                    encoder_node ─> wheel_ticks_std ─> wheel_odometry_node ─> odom
                                    yb_mra02_imu ─> imu ──────────────────────────┐
                                    odom ─────────────────────────────────────────┤─> ekf ─> TF odom->base_link
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('raspi_car_bringup')
    base_share = get_package_share_directory('raspi_car_base')
    sensors_share = get_package_share_directory('raspi_car_sensors')

    default_base_config = os.path.join(base_share, 'config', 'l298n.yaml')
    base_launch = os.path.join(base_share, 'launch', 'l298n_base.launch.py')
    imu_launch = os.path.join(sensors_share, 'launch', 'yb_mra02_uart.launch.py')
    ekf_config = os.path.join(bringup_share, 'config', 'ekf.yaml')

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car01'),
        DeclareLaunchArgument('base_config', default_value=default_base_config),

        # priority mux -> cmd_vel (consumed by motor node)
        Node(
            package='raspi_car_bringup', executable='cmd_vel_mux_node',
            name='cmd_vel_mux', namespace=namespace, output='screen',
            parameters=[{'output_topic': 'cmd_vel'}],
        ),
        # turn-assist on the manual path
        Node(
            package='raspi_car_bringup', executable='turn_assist_node',
            name='turn_assist', namespace=namespace, output='screen',
        ),
        # base: encoders + wheel odom + PID motor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'namespace': namespace,
                'base_config': LaunchConfiguration('base_config'),
                'cmd_vel_topic': 'cmd_vel',
            }.items(),
        ),
        # IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
            launch_arguments={'namespace': namespace}.items(),
        ),
        # EKF: fuse encoder odom + IMU -> owns odom->base_link TF
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', namespace=namespace, output='screen',
            parameters=[ekf_config],
        ),
        # static TFs: base_link -> laser_frame, base_link -> imu_link
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='laser_static_tf', namespace=namespace, output='screen',
            arguments=['0.12', '0.0', '0.10', '0', '0', '0', 'base_link', 'laser_frame'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='imu_static_tf', namespace=namespace, output='screen',
            arguments=['0.0', '0.0', '0.06', '0', '0', '0', 'base_link', 'imu_link'],
        ),
    ])
