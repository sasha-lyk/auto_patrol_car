"""Fixed-route patrol using Nav2 FollowWaypoints.

Loops through waypoints (from yaml) forever. Relies on AMCL auto-init
(set_initial_pose in nav2 params). With encoder+IMU EKF odometry, the
odom->base_link transform is now accurate enough that AMCL stays locked and
laps repeat reliably.
"""

import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def load_waypoints(path, logger):
    try:
        with open(path) as fh:
            data = yaml.safe_load(fh) or {}
        return data.get('waypoints', [])
    except Exception as exc:  # noqa: BLE001
        logger.error('failed to read waypoints %s: %s' % (path, exc))
        return []


def main(args=None):
    rclpy.init(args=args)
    boot = Node('patrol_bootstrap')
    boot.declare_parameter('waypoints_file', '')
    boot.declare_parameter('loop', True)
    boot.declare_parameter('nav_namespace', 'car01')
    waypoints_file = str(boot.get_parameter('waypoints_file').value)
    loop = bool(boot.get_parameter('loop').value)
    ns = str(boot.get_parameter('nav_namespace').value)
    wp_defs = load_waypoints(waypoints_file, boot.get_logger())
    boot.get_logger().info('patrol: %d waypoints, loop=%s, ns=%s'
                           % (len(wp_defs), loop, ns))
    boot.destroy_node()

    if not wp_defs:
        print('patrol: no waypoints, exiting')
        rclpy.shutdown()
        return

    nav = BasicNavigator(namespace=ns)
    nav.get_logger().info('patrol: waiting for Nav2...')
    nav.waitUntilNav2Active()

    poses = []
    for wp in wp_defs:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = float(wp['x'])
        p.pose.position.y = float(wp['y'])
        qx, qy, qz, qw = yaw_to_quat(float(wp.get('yaw', 0.0)))
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        poses.append(p)

    lap = 0
    try:
        while rclpy.ok():
            lap += 1
            nav.get_logger().info('patrol: lap %d' % lap)
            nav.followWaypoints(poses)
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb is not None:
                    nav.get_logger().info('patrol: waypoint %d/%d'
                                          % (fb.current_waypoint + 1, len(poses)))
                rclpy.spin_once(nav, timeout_sec=1.0)
            nav.get_logger().info('patrol: lap %d result=%s' % (lap, nav.getResult()))
            if not loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
