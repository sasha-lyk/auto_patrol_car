"""Turn-assist: convert 'spin-in-place' web/teleop commands into feasible arcs.

The car is heavily loaded and stalls on pure rotation. When a manual command
asks for angular motion with ~zero linear, inject a small forward speed so the
motion becomes an arc both wheels can actually execute. Sits ONLY on the
manual path (cmd_vel_web -> cmd_vel_web_assisted). Nav2 turning feasibility is
handled by the planner (min turning radius), not here.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurnAssistNode(Node):
    def __init__(self):
        super().__init__('turn_assist')
        self.declare_parameter('input_topic', 'cmd_vel_web')
        self.declare_parameter('output_topic', 'cmd_vel_web_assisted')
        self.declare_parameter('angular_threshold', 0.05)
        self.declare_parameter('linear_deadband', 0.03)
        self.declare_parameter('assist_linear_speed', 0.18)
        self.declare_parameter('default_direction', 'forward')

        self.angular_threshold = float(self.get_parameter('angular_threshold').value)
        self.linear_deadband = float(self.get_parameter('linear_deadband').value)
        self.assist_linear_speed = float(self.get_parameter('assist_linear_speed').value)
        self.default_direction = str(self.get_parameter('default_direction').value)

        self.pub = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10)
        self.create_subscription(
            Twist, str(self.get_parameter('input_topic').value), self.callback, 10)
        self.get_logger().info('turn_assist: %s -> %s' % (
            self.get_parameter('input_topic').value,
            self.get_parameter('output_topic').value))

    def callback(self, msg):
        out = Twist()
        out.linear.x = msg.linear.x
        out.angular.z = msg.angular.z
        turning = abs(msg.angular.z) > self.angular_threshold
        moving = abs(msg.linear.x) > self.linear_deadband
        if turning and not moving:
            sign = 1.0 if self.default_direction == 'forward' else -1.0
            out.linear.x = sign * self.assist_linear_speed
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TurnAssistNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
