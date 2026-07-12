"""Closed-loop wheel odometry from quadrature encoders.

Replaces the old open-loop integrator (which integrated the *commanded*
cmd_vel and therefore drifted on every wheel slip / stall). This node
integrates the *measured* wheel motion reported by encoder_node.

Inputs:
    <ns>/wheel_ticks_std   std_msgs/Int32MultiArray [left_ticks, right_ticks]

Outputs:
    <ns>/odom              nav_msgs/Odometry   (measured pose + twist)
    TF  odom -> base_link  (only if publish_tf and no EKF is running;
                            when EKF is used, set publish_tf=false and let
                            robot_localization own the odom->base_link TF)

Differential-drive forward kinematics (exact arc integration):
    d_left  = delta_left_ticks  * meters_per_tick
    d_right = delta_right_ticks * meters_per_tick
    d_center = (d_left + d_right) / 2
    d_theta  = (d_right - d_left) / wheel_separation
    if |d_theta| < eps:   # straight line
        x += d_center * cos(theta)
        y += d_center * sin(theta)
    else:                 # arc of radius R = d_center / d_theta
        R = d_center / d_theta
        x += R * (sin(theta + d_theta) - sin(theta))
        y += -R * (cos(theta + d_theta) - cos(theta))
    theta += d_theta

The pose/twist covariances are published so robot_localization's EKF can
weight this source against the IMU.
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster

from .control_core import DifferentialOdometry


def quaternion_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('ticks_topic', 'wheel_ticks_std')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)  # EKF owns TF by default
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_separation', 0.20)
        self.declare_parameter('encoder_cpr', 11)
        self.declare_parameter('gear_ratio', 90.0)
        self.declare_parameter('publish_rate', 50.0)

        self.odom_frame = str(self.get_parameter('odom_frame_id').value)
        self.base_frame = str(self.get_parameter('base_frame_id').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_sep = float(self.get_parameter('wheel_separation').value)
        cpr = int(self.get_parameter('encoder_cpr').value)
        gear = float(self.get_parameter('gear_ratio').value)

        ticks_per_wheel_rev = 4.0 * cpr * gear
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / ticks_per_wheel_rev

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.integrator = DifferentialOdometry(self.wheel_sep)
        self.have_prev = False
        self.prev_left = 0
        self.prev_right = 0
        self.prev_stamp = self.get_clock().now()

        self.v = 0.0
        self.w = 0.0

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(
            Int32MultiArray, str(self.get_parameter('ticks_topic').value),
            self.on_ticks, 20)

        self.get_logger().info(
            'wheel_odometry ready: m/tick=%.6f, wheel_sep=%.3f, publish_tf=%s'
            % (self.meters_per_tick, self.wheel_sep, self.publish_tf))

    def on_ticks(self, msg):
        if len(msg.data) < 2:
            return
        left = int(msg.data[0])
        right = int(msg.data[1])
        now = self.get_clock().now()

        if not self.have_prev:
            self.prev_left = left
            self.prev_right = right
            self.prev_stamp = now
            self.have_prev = True
            return

        dt = (now - self.prev_stamp).nanoseconds * 1e-9
        if dt <= 1e-6:
            return

        d_left = (left - self.prev_left) * self.meters_per_tick
        d_right = (right - self.prev_right) * self.meters_per_tick
        self.prev_left = left
        self.prev_right = right
        self.prev_stamp = now

        self.x, self.y, self.theta, d_center, d_theta = self.integrator.update(
            d_left, d_right)

        self.v = d_center / dt
        self.w = d_theta / dt

        self.publish_odom(now)

    def publish_odom(self, stamp):
        stamp_msg = stamp.to_msg()
        q = quaternion_from_yaw(self.theta)

        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        # Encoder odometry is trustworthy on x and yaw-rate, weak on y (a
        # diff-drive robot has no lateral motion, so y variance is small but
        # accumulated heading error leaks into it -> moderate values).
        odom.pose.covariance[0] = 0.002    # x
        odom.pose.covariance[7] = 0.002    # y
        odom.pose.covariance[35] = 0.02    # yaw
        odom.twist.covariance[0] = 0.005   # vx
        odom.twist.covariance[35] = 0.02   # vyaw
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp_msg
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
