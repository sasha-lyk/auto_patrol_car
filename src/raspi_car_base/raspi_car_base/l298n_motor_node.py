"""L298N differential-drive motor node with per-wheel PID velocity control.

Difference from the open-loop version:
    OLD:  cmd_vel -> wheel target speed -> static speed->PWM curve -> motor
          (no feedback: load, battery sag and slip all corrupt real speed)

    NEW:  cmd_vel -> wheel target speed (m/s)
          measured wheel speed (m/s) from encoder_node  --.
                                                          |-> PID -> PWM
          target - measured = error --------------------'
          -> the wheel actually reaches the commanded speed regardless of
             load / battery voltage, and the encoder odometry downstream sees
             the *true* motion. This is the inner control loop that makes the
             whole SLAM/Nav stack behave.

Inputs:
    <ns>/cmd_vel            geometry_msgs/Twist       (from cmd_vel mux)
    <ns>/wheel_speeds_std   std_msgs/Float32MultiArray [v_left, v_right] m/s

Outputs:
    L298N GPIO PWM + direction pins
    <ns>/base_controller/status  std_msgs/String (JSON telemetry for web)
"""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

from .control_core import Pid, wheel_targets


class L298NMotorDriver:
    def __init__(self, node):
        self.node = node
        self.dry_run = bool(node.get_parameter('dry_run').value)
        self.left_pwm_pin = int(node.get_parameter('left_pwm_pin_bcm').value)
        self.left_in1_pin = int(node.get_parameter('left_in1_pin_bcm').value)
        self.left_in2_pin = int(node.get_parameter('left_in2_pin_bcm').value)
        self.right_pwm_pin = int(node.get_parameter('right_pwm_pin_bcm').value)
        self.right_in1_pin = int(node.get_parameter('right_in1_pin_bcm').value)
        self.right_in2_pin = int(node.get_parameter('right_in2_pin_bcm').value)
        self.invert_left = bool(node.get_parameter('invert_left').value)
        self.invert_right = bool(node.get_parameter('invert_right').value)
        self.left_pwm = self.left_in1 = self.left_in2 = None
        self.right_pwm = self.right_in1 = self.right_in2 = None

        if self.dry_run:
            node.get_logger().warn('dry_run enabled: GPIO output disabled')
            return
        try:
            from gpiozero import DigitalOutputDevice, PWMOutputDevice
            self.left_pwm = PWMOutputDevice(self.left_pwm_pin, frequency=1000, initial_value=0.0)
            self.left_in1 = DigitalOutputDevice(self.left_in1_pin, initial_value=False)
            self.left_in2 = DigitalOutputDevice(self.left_in2_pin, initial_value=False)
            self.right_pwm = PWMOutputDevice(self.right_pwm_pin, frequency=1000, initial_value=0.0)
            self.right_in1 = DigitalOutputDevice(self.right_in1_pin, initial_value=False)
            self.right_in2 = DigitalOutputDevice(self.right_in2_pin, initial_value=False)
        except Exception as exc:  # noqa: BLE001
            self.dry_run = True
            node.get_logger().error('GPIO init failed, dry_run: %s' % exc)

    def set_wheel_pwm(self, left_value, right_value):
        self._set_one_side(left_value, self.invert_left, self.left_pwm,
                           self.left_in1, self.left_in2)
        self._set_one_side(right_value, self.invert_right, self.right_pwm,
                           self.right_in1, self.right_in2)

    def stop(self):
        self.set_wheel_pwm(0.0, 0.0)

    def close(self):
        self.stop()
        for d in (self.left_pwm, self.left_in1, self.left_in2,
                  self.right_pwm, self.right_in1, self.right_in2):
            if d is not None:
                d.close()

    def _set_one_side(self, value, inverted, pwm_device, fwd, bwd):
        output = -value if inverted else value
        duty = min(1.0, max(0.0, abs(output)))
        if self.dry_run:
            return
        if output > 0:
            fwd.on(); bwd.off(); pwm_device.value = duty
        elif output < 0:
            fwd.off(); bwd.on(); pwm_device.value = duty
        else:
            pwm_device.value = 0.0; fwd.off(); bwd.off()


class L298NMotorNode(Node):
    def __init__(self):
        super().__init__('l298n_motor_node')
        self._declare_parameters()

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.min_pwm = float(self.get_parameter('min_pwm').value)
        self.deadband_pwm = float(self.get_parameter('deadband_pwm').value)
        self.reverse_forward = bool(self.get_parameter('reverse_forward').value)
        self.cmd_vel_timeout_sec = float(self.get_parameter('cmd_vel_timeout_sec').value)
        self.status_period_sec = float(self.get_parameter('status_period_sec').value)
        self.closed_loop = bool(self.get_parameter('closed_loop').value)
        self.feedback_timeout = float(self.get_parameter('feedback_timeout_sec').value)
        self.estop_latched = False

        self.driver = L298NMotorDriver(self)

        self.pid_left = Pid(
            self.get_parameter('pid_kp').value,
            self.get_parameter('pid_ki').value,
            self.get_parameter('pid_kd').value,
            i_max=self.get_parameter('pid_i_max').value)
        self.pid_right = Pid(
            self.get_parameter('pid_kp').value,
            self.get_parameter('pid_ki').value,
            self.get_parameter('pid_kd').value,
            i_max=self.get_parameter('pid_i_max').value)

        self.target_left = 0.0
        self.target_right = 0.0
        self.meas_left = 0.0
        self.meas_right = 0.0
        self.left_pwm = 0.0
        self.right_pwm = 0.0
        self.last_cmd_time = time.monotonic()
        self.last_fb_time = 0.0
        self.last_ctrl_time = time.monotonic()
        self.timed_out = False
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        speeds_topic = str(self.get_parameter('wheel_speeds_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        estop_state_topic = str(self.get_parameter('emergency_state_topic').value)
        control_period = float(self.get_parameter('control_period_sec').value)

        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Float32MultiArray, speeds_topic, self.speeds_callback, 20)
        self.create_subscription(Bool, estop_state_topic, self.estop_callback, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_timer(control_period, self.control_loop)
        self.create_timer(self.status_period_sec, self.status_loop)

        self.get_logger().info(
            'L298N base ready: closed_loop=%s cmd_vel=%s speeds=%s'
            % (self.closed_loop, cmd_vel_topic, speeds_topic))

    def _declare_parameters(self):
        self.declare_parameter('dry_run', False)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('wheel_speeds_topic', 'wheel_speeds_std')
        self.declare_parameter('status_topic', 'base_controller/status')
        self.declare_parameter('emergency_state_topic', 'emergency_stop_latched')
        self.declare_parameter('left_pwm_pin_bcm', 18)
        self.declare_parameter('left_in1_pin_bcm', 23)
        self.declare_parameter('left_in2_pin_bcm', 24)
        self.declare_parameter('right_pwm_pin_bcm', 13)
        self.declare_parameter('right_in1_pin_bcm', 27)
        self.declare_parameter('right_in2_pin_bcm', 22)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('max_linear_speed', 0.60)
        self.declare_parameter('max_angular_speed', 3.0)
        self.declare_parameter('max_wheel_speed', 0.90)
        self.declare_parameter('min_pwm', 0.45)
        self.declare_parameter('deadband_pwm', 0.03)
        self.declare_parameter('reverse_forward', True)
        self.declare_parameter('cmd_vel_timeout_sec', 0.6)
        self.declare_parameter('control_period_sec', 0.02)
        self.declare_parameter('status_period_sec', 0.2)
        # closed-loop PID
        self.declare_parameter('closed_loop', True)
        self.declare_parameter('feedback_timeout_sec', 0.3)
        self.declare_parameter('pid_kp', 2.2)
        self.declare_parameter('pid_ki', 6.0)
        self.declare_parameter('pid_kd', 0.02)
        self.declare_parameter('pid_i_max', 0.6)
        # feed-forward: nominal PWM to reach max_wheel_speed (helps PID start)
        self.declare_parameter('feedforward_gain', 0.9)

    def cmd_vel_callback(self, msg):
        if self.estop_latched:
            return
        linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, msg.angular.z))
        if self.reverse_forward:
            linear_x = -linear_x
        self.current_linear_x = linear_x
        self.current_angular_z = angular_z
        self.last_cmd_time = time.monotonic()
        self.timed_out = False

        self.target_left, self.target_right = wheel_targets(
            linear_x, angular_z, self.wheel_base)

    def speeds_callback(self, msg):
        if len(msg.data) >= 2:
            self.meas_left = float(msg.data[0])
            self.meas_right = float(msg.data[1])
            self.last_fb_time = time.monotonic()

    def estop_callback(self, msg):
        was_latched = self.estop_latched
        self.estop_latched = bool(msg.data)
        if self.estop_latched:
            self.target_left = 0.0
            self.target_right = 0.0
            self.left_pwm = 0.0
            self.right_pwm = 0.0
            self.pid_left.reset()
            self.pid_right.reset()
            self.driver.stop()
            if not was_latched:
                self.get_logger().error('motor output inhibited by latched E-STOP')
        elif was_latched:
            self.last_cmd_time = time.monotonic()
            self.get_logger().warn('motor E-STOP inhibit cleared')

    def control_loop(self):
        now = time.monotonic()
        dt = now - self.last_ctrl_time
        self.last_ctrl_time = now

        # Defence in depth: even if the mux is misconfigured, a latched state
        # directly inhibits the GPIO motor output.
        if self.estop_latched:
            self.driver.stop()
            return

        # cmd_vel watchdog: stop if no command recently
        if now - self.last_cmd_time > self.cmd_vel_timeout_sec:
            if not self.timed_out:
                self.get_logger().warn('cmd_vel timeout, stopping')
                self.timed_out = True
            self.target_left = 0.0
            self.target_right = 0.0
            self.pid_left.reset()
            self.pid_right.reset()
            self.left_pwm = 0.0
            self.right_pwm = 0.0
            self.driver.stop()
            return

        fb_ok = self.closed_loop and (now - self.last_fb_time) <= self.feedback_timeout

        if fb_ok:
            # PID + feed-forward
            ff = float(self.get_parameter('feedforward_gain').value)
            ff_left = ff * self.target_left / self.max_wheel_speed
            ff_right = ff * self.target_right / self.max_wheel_speed
            u_left = ff_left + self.pid_left.step(self.target_left, self.meas_left, dt)
            u_right = ff_right + self.pid_right.step(self.target_right, self.meas_right, dt)
            self.left_pwm = self._apply_deadband(u_left)
            self.right_pwm = self._apply_deadband(u_right)
        else:
            # fall back to open-loop speed->PWM curve if encoders are missing
            if self.closed_loop:
                self.get_logger().warn(
                    'encoder feedback stale -> open-loop fallback',
                    throttle_duration_sec=2.0)
            self.left_pwm = self._speed_to_pwm(self.target_left)
            self.right_pwm = self._speed_to_pwm(self.target_right)

        self.driver.set_wheel_pwm(self.left_pwm, self.right_pwm)

    def _apply_deadband(self, u):
        u = max(-1.0, min(1.0, u))
        if abs(u) < self.deadband_pwm:
            return 0.0
        # lift above L298N minimum move PWM while preserving sign
        sign = 1.0 if u > 0 else -1.0
        mag = self.min_pwm + (1.0 - self.min_pwm) * abs(u)
        return sign * min(1.0, mag)

    def _speed_to_pwm(self, wheel_speed):
        normalized = max(-1.0, min(1.0, wheel_speed / self.max_wheel_speed))
        if abs(normalized) < self.deadband_pwm:
            return 0.0
        sign = 1.0 if normalized > 0 else -1.0
        duty = self.min_pwm + (1.0 - self.min_pwm) * abs(normalized)
        return sign * min(1.0, max(0.0, duty))

    def status_loop(self):
        msg = String()
        msg.data = json.dumps({
            'type': 'base_status',
            'closed_loop': self.closed_loop,
            'linear_x': self.current_linear_x,
            'angular_z': self.current_angular_z,
            'target_left': round(self.target_left, 3),
            'target_right': round(self.target_right, 3),
            'meas_left': round(self.meas_left, 3),
            'meas_right': round(self.meas_right, 3),
            'left_pwm': round(self.left_pwm, 3),
            'right_pwm': round(self.right_pwm, 3),
            'timed_out': self.timed_out,
            'dry_run': self.driver.dry_run,
            'estop_latched': self.estop_latched,
        }, separators=(',', ':'))
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = L298NMotorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
