"""Validated fixed-route patrol with retries, E-STOP pause and run metrics."""

import json
import math
import os
import time
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from std_msgs.msg import Bool

from .route_core import RouteValidationError, load_route


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def append_metric(path, record):
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'a', encoding='utf-8') as stream:
        stream.write(json.dumps(record, ensure_ascii=False, separators=(',', ':')) + '\n')


def make_poses(nav, waypoints):
    poses = []
    for waypoint in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = nav.get_clock().now().to_msg()
        pose.pose.position.x = waypoint['x']
        pose.pose.position.y = waypoint['y']
        qx, qy, qz, qw = yaw_to_quat(waypoint['yaw'])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        poses.append(pose)
    return poses


def main(args=None):
    rclpy.init(args=args)
    boot = Node('patrol_bootstrap')
    boot.declare_parameter('waypoints_file', '~/.ros/raspi_car/routes/room1_patrol.yaml')
    boot.declare_parameter('map_yaml', '')
    boot.declare_parameter('loop', True)
    boot.declare_parameter('nav_namespace', 'car01')
    boot.declare_parameter('max_lap_retries', 2)
    boot.declare_parameter('max_consecutive_failures', 3)
    boot.declare_parameter('retry_delay_sec', 3.0)
    boot.declare_parameter('lap_pause_sec', 1.0)
    boot.declare_parameter(
        'metrics_file', '~/.ros/raspi_car/logs/patrol_metrics.jsonl')

    route_path = str(boot.get_parameter('waypoints_file').value)
    map_yaml = str(boot.get_parameter('map_yaml').value)
    loop = bool(boot.get_parameter('loop').value)
    namespace = str(boot.get_parameter('nav_namespace').value)
    max_retries = int(boot.get_parameter('max_lap_retries').value)
    max_failures = int(boot.get_parameter('max_consecutive_failures').value)
    retry_delay = float(boot.get_parameter('retry_delay_sec').value)
    lap_pause = float(boot.get_parameter('lap_pause_sec').value)
    metrics_file = str(boot.get_parameter('metrics_file').value)
    try:
        route = load_route(route_path, map_yaml=map_yaml, require_calibrated=True)
    except (OSError, RouteValidationError) as exc:
        boot.get_logger().fatal('patrol route rejected: %s' % exc)
        boot.destroy_node()
        rclpy.shutdown()
        return
    boot.get_logger().info(
        'validated route %s: %d waypoints, map fingerprint OK'
        % (route['route'].get('name', 'unnamed'), len(route['waypoints'])))
    boot.destroy_node()

    nav = BasicNavigator(namespace=namespace)
    estop = {'latched': False}

    def on_estop(msg):
        estop['latched'] = bool(msg.data)

    nav.create_subscription(Bool, 'emergency_stop_latched', on_estop, 10)
    nav.get_logger().info('patrol: waiting for Nav2 lifecycle nodes...')
    nav.waitUntilNav2Active()
    poses = make_poses(nav, route['waypoints'])
    lap = 0
    consecutive_failures = 0

    try:
        while rclpy.ok():
            if estop['latched']:
                nav.get_logger().warn('patrol paused: E-STOP is latched')
                while rclpy.ok() and estop['latched']:
                    rclpy.spin_once(nav, timeout_sec=0.25)
                nav.get_logger().info('E-STOP reset; patrol may resume')

            lap += 1
            lap_succeeded = False
            lap_started = time.monotonic()
            result_name = 'UNKNOWN'
            for attempt in range(max_retries + 1):
                for pose in poses:
                    pose.header.stamp = nav.get_clock().now().to_msg()
                nav.get_logger().info(
                    'patrol lap %d attempt %d/%d' % (lap, attempt + 1, max_retries + 1))
                nav.followWaypoints(poses)
                cancelled_by_estop = False
                while not nav.isTaskComplete():
                    if estop['latched']:
                        nav.get_logger().error('E-STOP during patrol; cancelling Nav2 task')
                        nav.cancelTask()
                        cancelled_by_estop = True
                        break
                    feedback = nav.getFeedback()
                    if feedback is not None:
                        nav.get_logger().info(
                            'patrol waypoint %d/%d'
                            % (feedback.current_waypoint + 1, len(poses)),
                            throttle_duration_sec=2.0)
                result = nav.getResult()
                result_name = getattr(result, 'name', str(result))
                if result == TaskResult.SUCCEEDED:
                    lap_succeeded = True
                    break
                if cancelled_by_estop:
                    while rclpy.ok() and estop['latched']:
                        rclpy.spin_once(nav, timeout_sec=0.25)
                if attempt < max_retries:
                    nav.get_logger().warn(
                        'lap failed (%s); retrying after %.1fs' % (result_name, retry_delay))
                    time.sleep(retry_delay)

            duration = round(time.monotonic() - lap_started, 3)
            append_metric(metrics_file, {
                'timestamp': datetime.now(timezone.utc).isoformat(),
                'route': route['route'].get('name', 'unnamed'),
                'lap': lap,
                'success': lap_succeeded,
                'result': result_name,
                'duration_sec': duration,
                'waypoint_count': len(poses),
            })
            if lap_succeeded:
                consecutive_failures = 0
                nav.get_logger().info('patrol lap %d succeeded in %.1fs' % (lap, duration))
            else:
                consecutive_failures += 1
                nav.get_logger().error(
                    'patrol lap %d failed; consecutive failures=%d'
                    % (lap, consecutive_failures))
                if consecutive_failures >= max_failures:
                    nav.get_logger().fatal(
                        'failure threshold reached; stopping patrol for operator inspection')
                    break
            if not loop:
                break
            time.sleep(lap_pause)
    except KeyboardInterrupt:
        nav.cancelTask()
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
