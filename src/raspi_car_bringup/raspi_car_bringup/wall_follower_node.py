"""Reactive right-wall following (no map, no localization, no odometry).

Pure laser-reactive perimeter patrol. Immune to wheel slip because it only
reacts to the CURRENT scan. Used as a fallback perimeter patrol when the map/
localization stack is not running. The car cannot rotate in place, so all
turns are arcs (forward speed kept while turning).

The LIDAR is mounted rotated: car FORWARD (+x_base) = laser angle
`laser_front_angle_deg` (default -85 deg, calibrated). Output -> cmd_vel_nav
(mux nav source; web/emergency override anytime).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def _norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.declare_parameter('laser_front_angle_deg', -85.0)
        self.declare_parameter('scan_topic', '/car01/scan')
        self.declare_parameter('cmd_topic', 'cmd_vel_nav')
        self.declare_parameter('wall_side', 'right')
        self.declare_parameter('target_wall_dist', 0.6)
        self.declare_parameter('cruise_speed', 0.18)
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('max_ang', 1.2)
        self.declare_parameter('front_block_dist', 0.6)
        self.declare_parameter('lost_wall_dist', 1.7)
        self.declare_parameter('estop_dist', 0.25)
        self.declare_parameter('sector_half_deg', 20.0)
        self.declare_parameter('enabled', True)
        self.declare_parameter('angular_sign', -1.0)
        self.declare_parameter('backup_speed', 0.14)
        self.declare_parameter('backup_time', 1.2)
        self.declare_parameter('backup_turn', 0.4)
        self.declare_parameter('turnout_time', 2.2)
        self.declare_parameter('turnout_turn', 0.8)
        self.declare_parameter('scan_timeout', 0.5)

        g = self.get_parameter
        self.front = float(g('laser_front_angle_deg').value) * math.pi / 180.0
        self.side = str(g('wall_side').value)
        self.sgn = -1.0 if self.side == 'right' else 1.0
        self.target = float(g('target_wall_dist').value)
        self.cruise = float(g('cruise_speed').value)
        self.kp = float(g('kp').value)
        self.max_ang = float(g('max_ang').value)
        self.front_block = float(g('front_block_dist').value)
        self.lost_wall = float(g('lost_wall_dist').value)
        self.estop = float(g('estop_dist').value)
        self.half = float(g('sector_half_deg').value) * math.pi / 180.0
        self.enabled = bool(g('enabled').value)
        self.asign = float(g('angular_sign').value)
        self.backup_speed = float(g('backup_speed').value)
        self.backup_time = float(g('backup_time').value)
        self.backup_turn = float(g('backup_turn').value)
        self.turnout_time = float(g('turnout_time').value)
        self.turnout_turn = float(g('turnout_turn').value)
        self.scan_timeout = float(g('scan_timeout').value)

        import time as _t
        self._t = _t
        self.backup_until = 0.0
        self.turnout_until = 0.0

        self.pub = self.create_publisher(Twist, str(g('cmd_topic').value), 10)
        self.create_subscription(LaserScan, str(g('scan_topic').value), self.on_scan, 10)
        self.last_scan_time = self._t.monotonic()
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info('wall_follower ready: side=%s front@%.0fdeg'
                               % (self.side, math.degrees(self.front)))

    def _sector_min(self, scan, center_angle, half):
        best = float('inf')
        for i in range(len(scan.ranges)):
            a = scan.angle_min + i * scan.angle_increment
            if abs(_norm(a - center_angle)) <= half:
                r = scan.ranges[i]
                if math.isfinite(r) and scan.range_min < r < scan.range_max and r < best:
                    best = r
        return best

    def _watchdog(self):
        if self._t.monotonic() - self.last_scan_time > self.scan_timeout:
            self.pub.publish(Twist())
            self.get_logger().error('scan STALE -> STOP', throttle_duration_sec=2.0)

    def on_scan(self, scan):
        self.last_scan_time = self._t.monotonic()
        if not self.enabled:
            return
        side_ang = _norm(self.front + self.sgn * math.pi / 2.0)
        frontside_ang = _norm(self.front + self.sgn * math.pi / 4.0)
        d_front = self._sector_min(scan, self.front, self.half)
        d_side = self._sector_min(scan, side_ang, self.half)
        d_fs = self._sector_min(scan, frontside_ang, self.half)

        cmd = Twist()
        now = self._t.monotonic()

        if now < self.backup_until:
            cmd.linear.x = -self.backup_speed
            cmd.angular.z = self.asign * (-self.sgn * self.backup_turn)
            self.pub.publish(cmd)
            return

        if now < self.turnout_until:
            if d_front < self.estop:
                self.backup_until = now + self.backup_time
                self.turnout_until = self.backup_until + self.turnout_time
                self.pub.publish(Twist())
                return
            cmd.linear.x = self.cruise
            cmd.angular.z = self.asign * (-self.sgn * self.turnout_turn)
            self.pub.publish(cmd)
            return

        if d_front < self.front_block or d_fs < self.front_block * 0.8:
            self.backup_until = now + self.backup_time
            self.turnout_until = self.backup_until + self.turnout_time
            self.pub.publish(Twist())
            self.get_logger().warn('corner %.2fm -> backup+turnout' % d_front)
            return

        if d_side > self.lost_wall:
            cmd.linear.x = self.cruise * 0.7
            cmd.angular.z = self.asign * (self.sgn * self.max_ang * 0.7)
            self.pub.publish(cmd)
            return

        err = d_side - self.target
        ang = max(-self.max_ang, min(self.max_ang, self.sgn * self.kp * err))
        cmd.linear.x = self.cruise
        cmd.angular.z = self.asign * ang
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
