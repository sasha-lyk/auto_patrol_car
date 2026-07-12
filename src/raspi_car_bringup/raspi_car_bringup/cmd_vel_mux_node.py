"""Priority cmd_vel multiplexer.

emergency (255) > web (100) > teleop (90) > nav (50). Each source has a
timeout; when it goes stale it drops out and the next-highest active source
takes over. If nothing is active, publishes zero (stop). This is the single
choke point that guarantees emergency-stop always wins.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('emergency_topic', 'cmd_vel_emergency')
        self.declare_parameter('web_topic', 'cmd_vel_web_assisted')
        self.declare_parameter('teleop_topic', 'cmd_vel_teleop')
        self.declare_parameter('nav_topic', 'cmd_vel_nav')
        self.declare_parameter('emergency_timeout', 0.3)
        self.declare_parameter('web_timeout', 0.8)
        self.declare_parameter('teleop_timeout', 0.8)
        self.declare_parameter('nav_timeout', 0.6)
        self.declare_parameter('publish_rate', 20.0)

        self.sources = {
            'emergency': self._src('emergency_topic', 'emergency_timeout', 255),
            'web': self._src('web_topic', 'web_timeout', 100),
            'teleop': self._src('teleop_topic', 'teleop_timeout', 90),
            'nav': self._src('nav_topic', 'nav_timeout', 50),
        }
        self.publisher = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)
        for name, src in self.sources.items():
            self.create_subscription(Twist, src['topic'], self._make_cb(name), 10)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value),
                          self.timer_callback)
        self.last_active = ''
        self.get_logger().info('cmd_vel mux: emergency>web>teleop>nav -> cmd_vel')

    def _src(self, topic_param, timeout_param, priority):
        return {
            'topic': self.get_parameter(topic_param).value,
            'timeout': float(self.get_parameter(timeout_param).value),
            'priority': priority,
            'msg': Twist(),
            'stamp': 0.0,
        }

    def _make_cb(self, name):
        def cb(msg):
            self.sources[name]['msg'] = msg
            self.sources[name]['stamp'] = time.monotonic()
        return cb

    def timer_callback(self):
        now = time.monotonic()
        active_name = ''
        active = None
        for name, src in self.sources.items():
            if now - src['stamp'] > src['timeout']:
                continue
            if active is None or src['priority'] > active['priority']:
                active_name = name
                active = src
        if active is None:
            self.publisher.publish(Twist())
            if self.last_active:
                self.get_logger().info('mux idle -> stop')
                self.last_active = ''
            return
        self.publisher.publish(active['msg'])
        if active_name != self.last_active:
            self.get_logger().info('mux active: %s' % active_name)
            self.last_active = active_name


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
