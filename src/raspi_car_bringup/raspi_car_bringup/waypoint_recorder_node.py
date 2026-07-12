"""Capture field-calibrated patrol waypoints from the current AMCL pose."""

import math
import os
import tempfile
import time
from datetime import datetime, timezone

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_srvs.srv import Trigger

from .route_core import RouteValidationError, map_bundle_sha256, validate_route


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.declare_parameter('pose_topic', 'amcl_pose')
        self.declare_parameter('output_file', '~/.ros/raspi_car/routes/room1_patrol.yaml')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('route_name', 'room1_patrol')
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('max_xy_variance', 0.25)
        self.declare_parameter('max_yaw_variance', 0.30)
        self.declare_parameter('min_waypoint_spacing', 0.20)
        self.declare_parameter('clearance_m', 0.30)

        self.latest_pose = None
        self.latest_pose_time = 0.0
        self.waypoints = []
        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter('pose_topic').value,
            self._on_pose, 10)
        self.create_service(Trigger, 'capture_waypoint', self._capture)
        self.create_service(Trigger, 'save_route', self._save)
        self.create_service(Trigger, 'clear_route', self._clear)
        self.get_logger().info(
            'waypoint recorder ready: call capture_waypoint at each field position')

    def _on_pose(self, msg):
        self.latest_pose = msg
        self.latest_pose_time = time.monotonic()

    def _pose_error(self):
        if self.latest_pose is None:
            return 'no AMCL pose received'
        age = time.monotonic() - self.latest_pose_time
        if age > float(self.get_parameter('pose_timeout_sec').value):
            return 'AMCL pose is stale (%.2fs)' % age
        covariance = self.latest_pose.pose.covariance
        if covariance[0] > float(self.get_parameter('max_xy_variance').value):
            return 'AMCL x variance too high: %.3f' % covariance[0]
        if covariance[7] > float(self.get_parameter('max_xy_variance').value):
            return 'AMCL y variance too high: %.3f' % covariance[7]
        if covariance[35] > float(self.get_parameter('max_yaw_variance').value):
            return 'AMCL yaw variance too high: %.3f' % covariance[35]
        return ''

    def _capture(self, request, response):
        error = self._pose_error()
        if error:
            response.success = False
            response.message = error
            return response
        pose = self.latest_pose.pose.pose
        waypoint = {
            'x': round(float(pose.position.x), 4),
            'y': round(float(pose.position.y), 4),
            'yaw': round(yaw_from_quaternion(pose.orientation), 4),
        }
        if self.waypoints:
            previous = self.waypoints[-1]
            distance = math.hypot(waypoint['x'] - previous['x'], waypoint['y'] - previous['y'])
            if distance < float(self.get_parameter('min_waypoint_spacing').value):
                response.success = False
                response.message = 'too close to previous waypoint: %.3fm' % distance
                return response
        self.waypoints.append(waypoint)
        response.success = True
        response.message = 'captured waypoint %d: %s' % (len(self.waypoints), waypoint)
        self.get_logger().info(response.message)
        return response

    def _save(self, request, response):
        if len(self.waypoints) < 2:
            response.success = False
            response.message = 'capture at least two waypoints before saving'
            return response
        map_yaml = os.path.abspath(os.path.expanduser(
            str(self.get_parameter('map_yaml').value)))
        if not map_yaml or not os.path.isfile(map_yaml):
            response.success = False
            response.message = 'map_yaml does not exist: %s' % map_yaml
            return response
        output = os.path.abspath(os.path.expanduser(
            str(self.get_parameter('output_file').value)))
        document = {
            'route': {
                'name': str(self.get_parameter('route_name').value),
                'frame_id': 'map',
                'calibrated': True,
                'created_at': datetime.now(timezone.utc).isoformat(),
                'map_file': os.path.basename(map_yaml),
                'map_sha256': map_bundle_sha256(map_yaml),
                'clearance_m': float(self.get_parameter('clearance_m').value),
            },
            'waypoints': list(self.waypoints),
        }
        try:
            validate_route(document, map_yaml=map_yaml, require_calibrated=True)
        except RouteValidationError as exc:
            response.success = False
            response.message = 'route failed map safety validation: %s' % exc
            return response
        os.makedirs(os.path.dirname(output), exist_ok=True)
        handle, temporary = tempfile.mkstemp(
            prefix='.route-', suffix='.yaml', dir=os.path.dirname(output), text=True)
        try:
            with os.fdopen(handle, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(document, stream, sort_keys=False, allow_unicode=True)
            os.replace(temporary, output)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        response.success = True
        response.message = 'saved %d waypoints to %s' % (len(self.waypoints), output)
        self.get_logger().info(response.message)
        return response

    def _clear(self, request, response):
        count = len(self.waypoints)
        self.waypoints.clear()
        response.success = True
        response.message = 'cleared %d captured waypoints' % count
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
