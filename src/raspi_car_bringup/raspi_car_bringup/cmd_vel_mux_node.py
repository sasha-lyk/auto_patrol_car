"""Priority cmd_vel multiplexer with a latched emergency stop.

The emergency stop is deliberately NOT a velocity source with a timeout.
Once triggered it publishes zero forever, until a separate reset message is
received. Resetting also discards all cached commands so motion can resume
only after a source publishes a fresh command.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from .safety_core import LatchedEmergencyStop, VelocityArbiter


class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('emergency_stop_topic', 'emergency_stop')
        self.declare_parameter('emergency_reset_topic', 'emergency_stop_reset')
        self.declare_parameter('emergency_state_topic', 'emergency_stop_latched')
        self.declare_parameter('web_topic', 'cmd_vel_web_assisted')
        self.declare_parameter('teleop_topic', 'cmd_vel_teleop')
        self.declare_parameter('nav_topic', 'cmd_vel_nav')
        self.declare_parameter('web_timeout', 0.8)
        self.declare_parameter('teleop_timeout', 0.8)
        self.declare_parameter('nav_timeout', 0.6)
        self.declare_parameter('publish_rate', 20.0)

        self.source_topics = {
            'web': self.get_parameter('web_topic').value,
            'teleop': self.get_parameter('teleop_topic').value,
            'nav': self.get_parameter('nav_topic').value,
        }
        self.arbiter = VelocityArbiter({
            'web': {'timeout': self.get_parameter('web_timeout').value, 'priority': 100},
            'teleop': {'timeout': self.get_parameter('teleop_timeout').value, 'priority': 90},
            'nav': {'timeout': self.get_parameter('nav_timeout').value, 'priority': 50},
        })
        self.estop = LatchedEmergencyStop()
        self.publisher = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)
        for name, topic in self.source_topics.items():
            self.create_subscription(Twist, topic, self._make_cb(name), 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            Bool, self.get_parameter('emergency_state_topic').value, state_qos)
        self.create_subscription(
            Bool, self.get_parameter('emergency_stop_topic').value,
            self._on_emergency_stop, 10)
        self.create_subscription(
            Bool, self.get_parameter('emergency_reset_topic').value,
            self._on_emergency_reset, 10)
        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value),
                          self.timer_callback)
        self.last_active = ''
        self._publish_estop_state()
        self.get_logger().info('cmd_vel mux: LATCHED E-STOP > web > teleop > nav')

    def _make_cb(self, name):
        def cb(msg):
            self.arbiter.update(name, msg)
        return cb

    def _publish_estop_state(self):
        msg = Bool()
        msg.data = self.estop.latched
        self.state_pub.publish(msg)

    def _on_emergency_stop(self, msg):
        if not msg.data:
            return
        if self.estop.trigger():
            self.get_logger().error('EMERGENCY STOP LATCHED - explicit reset required')
        self._publish_estop_state()

    def _on_emergency_reset(self, msg):
        if not msg.data:
            return
        if self.estop.reset():
            self.arbiter.clear()
            self.get_logger().warn('emergency stop reset; waiting for a fresh command')
        self._publish_estop_state()

    def timer_callback(self):
        if self.estop.latched:
            self.publisher.publish(Twist())
            if self.last_active != 'emergency_stop':
                self.get_logger().error('mux active: emergency_stop (latched)')
                self.last_active = 'emergency_stop'
            return
        active_name, message = self.arbiter.select()
        if message is None:
            self.publisher.publish(Twist())
            if self.last_active:
                self.get_logger().info('mux idle -> stop')
                self.last_active = ''
            return
        self.publisher.publish(message)
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
