"""ROS2 web backend for the encoder car.

Two directions:
  Browser --HTTP /api/cmd--> Twist on /<ns>/cmd_vel_web (-> turn_assist -> mux)
  ROS2 topics --subscribe--> /api/status returns REAL telemetry:
      /<ns>/odom                 -> pose (x,y,yaw) + speed from EKF/encoders
      /<ns>/imu                  -> roll/pitch/yaw, gyro
      /<ns>/base_controller/status -> per-wheel target vs measured speed, PWM

Unlike the old backend (which returned hard-coded zeros), this one runs a
ROS2 executor in a background thread and reports live state, so the dashboard
actually reflects the closed-loop behaviour.
"""

import math
import os
import threading
import time
from datetime import datetime

import json as _json

from flask import Flask, jsonify, request, send_from_directory

try:
    from flask_cors import CORS
except ImportError:
    CORS = None

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


APP_DIR = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.environ.get(
    'WEB_DIR', os.path.normpath(os.path.join(APP_DIR, '..', 'web')))

CAR_NAMESPACE = os.environ.get('CAR_NAMESPACE', 'car01').strip().strip('/')
HTTP_PORT = int(os.environ.get('HTTP_PORT', '8080'))
LINEAR_SPEEDS = {
    1: float(os.environ.get('CAR_LOW_LINEAR', '0.12')),
    2: float(os.environ.get('CAR_MED_LINEAR', '0.20')),
    3: float(os.environ.get('CAR_HIGH_LINEAR', '0.30')),
}
ANGULAR_SPEEDS = {
    1: float(os.environ.get('CAR_LOW_ANGULAR', '0.7')),
    2: float(os.environ.get('CAR_MED_ANGULAR', '1.1')),
    3: float(os.environ.get('CAR_HIGH_ANGULAR', '1.5')),
}


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class WebBridge:
    """Holds the ROS2 node, publisher and latest telemetry snapshot."""

    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('web_cmd_bridge', namespace=CAR_NAMESPACE)
        self.cmd_pub = self.node.create_publisher(Twist, 'cmd_vel_web', 10)
        self.estop_pub = self.node.create_publisher(Twist, 'cmd_vel_emergency', 10)

        self.node.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.node.create_subscription(Imu, 'imu', self._on_imu, 10)
        self.node.create_subscription(String, 'base_controller/status',
                                      self._on_status, 10)

        self.lock = threading.Lock()
        self.state = {
            'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'v': 0.0, 'w': 0.0,
            'roll': 0.0, 'pitch': 0.0,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
            'base': {},
            'odom_age_ms': None, 'imu_age_ms': None, 'base_age_ms': None,
        }
        self._t_odom = 0.0
        self._t_imu = 0.0
        self._t_base = 0.0

        self.last_cmd = 'S'
        self.speed_level = 2
        self.last_cmd_time = 0.0
        self.start_time = time.time()

        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin.start()

    def _spin_loop(self):
        rclpy.spin(self.node)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        with self.lock:
            self.state['x'] = msg.pose.pose.position.x
            self.state['y'] = msg.pose.pose.position.y
            self.state['yaw'] = yaw_from_quat(q.x, q.y, q.z, q.w)
            self.state['v'] = msg.twist.twist.linear.x
            self.state['w'] = msg.twist.twist.angular.z
        self._t_odom = time.time()

    def _on_imu(self, msg):
        q = msg.orientation
        with self.lock:
            self.state['roll'] = math.atan2(
                2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
            self.state['pitch'] = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
            self.state['gx'] = msg.angular_velocity.x
            self.state['gy'] = msg.angular_velocity.y
            self.state['gz'] = msg.angular_velocity.z
        self._t_imu = time.time()

    def _on_status(self, msg):
        try:
            data = _json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        with self.lock:
            self.state['base'] = data
        self._t_base = time.time()

    def make_twist(self, cmd):
        twist = Twist()
        linear = LINEAR_SPEEDS[self.speed_level]
        angular = ANGULAR_SPEEDS[self.speed_level]
        if cmd == 'F':
            twist.linear.x = linear
        elif cmd == 'B':
            twist.linear.x = -linear
        elif cmd == 'L':
            twist.angular.z = angular
        elif cmd == 'R':
            twist.angular.z = -angular
        return twist

    def publish_cmd(self, cmd):
        self.cmd_pub.publish(self.make_twist(cmd))
        self.last_cmd = cmd
        self.last_cmd_time = time.time()

    def publish_estop(self):
        # zero twist on the highest-priority mux channel -> immediate stop
        self.estop_pub.publish(Twist())
        self.last_cmd = 'S'
        self.last_cmd_time = time.time()

    def snapshot(self):
        now = time.time()
        with self.lock:
            s = dict(self.state)
            base = dict(s.get('base', {}))
        def age(t):
            return None if t <= 0 else int((now - t) * 1000)
        return {
            'online': True,
            'ros2_bridge': True,
            'namespace': CAR_NAMESPACE,
            'cmd_topic': '/%s/cmd_vel_web' % CAR_NAMESPACE,
            'last_cmd': self.last_cmd,
            'dir': self.last_cmd,
            'speed': self.speed_level,
            'count': int(now - self.start_time),
            'x': round(s['x'], 3), 'y': round(s['y'], 3),
            'yaw': round(s['yaw'], 3),
            'v': round(s['v'], 3), 'w': round(s['w'], 3),
            'roll': round(s['roll'], 3), 'pitch': round(s['pitch'], 3),
            'gx': round(s['gx'], 3), 'gy': round(s['gy'], 3), 'gz': round(s['gz'], 3),
            'base': base,
            'odom_age_ms': age(self._t_odom),
            'imu_age_ms': age(self._t_imu),
            'base_age_ms': age(self._t_base),
            'update_time': datetime.now().strftime('%H:%M:%S'),
        }


app = Flask(__name__)
if CORS is not None:
    CORS(app)
bridge = None  # set in main()


@app.route('/')
def index():
    return send_from_directory(WEB_DIR, 'index.html')


@app.route('/<path:path>')
def static_files(path):
    return send_from_directory(WEB_DIR, path)


@app.route('/api/status')
def api_status():
    return jsonify(bridge.snapshot())


@app.route('/api/health')
def api_health():
    return jsonify({'ok': True, 'mode': 'ros2', 'namespace': CAR_NAMESPACE,
                    'uptime_sec': int(time.time() - bridge.start_time)})


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    body = request.get_json(silent=True) or {}
    cmd = str(body.get('c', '')).strip().upper()
    if cmd in ('1', '2', '3'):
        bridge.speed_level = int(cmd)
        return jsonify({'ok': True, 'cmd': cmd, 'speed': bridge.speed_level})
    if cmd == 'E':  # emergency stop
        bridge.publish_estop()
        return jsonify({'ok': True, 'cmd': 'E'})
    if cmd not in ('F', 'B', 'L', 'R', 'S'):
        return jsonify({'ok': False, 'error': 'invalid command'}), 400
    bridge.publish_cmd(cmd)
    return jsonify({'ok': True, 'cmd': cmd, 'speed': bridge.speed_level})


def main():
    global bridge
    bridge = WebBridge()
    print('ROS2 web backend: ns=/%s port=%d web=%s'
          % (CAR_NAMESPACE, HTTP_PORT, WEB_DIR))
    app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True)


if __name__ == '__main__':
    main()
