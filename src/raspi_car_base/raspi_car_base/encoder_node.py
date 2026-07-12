"""Quadrature wheel-encoder driver for the differential-drive base.

Reads two incremental quadrature encoders (left / right wheel) on the
Raspberry Pi GPIO and publishes:

    <ns>/wheel_ticks      raspi_car_interfaces/WheelTicks  (cumulative counts)
    <ns>/wheel_speeds     raspi_car_interfaces/WheelSpeeds (rad/s, m/s)

To avoid a hard dependency on a custom message package (so this can run on a
plain ROS2 install), the node publishes the same information on two standard
topics as well:

    <ns>/wheel_ticks_std  std_msgs/Int32MultiArray  [left_ticks, right_ticks]
    <ns>/wheel_speeds_std std_msgs/Float32MultiArray [v_left, v_right] (m/s)

The heavy lifting is edge counting. Each channel uses a full x4 quadrature
decode: both A and B edges are counted and the direction is taken from the
phase relationship between A and B. This gives 4 * CPR counts per motor
revolution, which -- after the gearbox reduction -- yields
ticks_per_wheel_rev = 4 * CPR * gear_ratio.

GPIO backends, in order of preference:
    1. lgpio        (Ubuntu 22.04 / Pi 5 / Pi 4, modern)
    2. RPi.GPIO     (classic)
    3. dry_run      (no hardware -- generates zero ticks, lets the graph run)

Encoder wiring (BCM, defaults -- override in l298n.yaml):
    left_a_pin  = 5    left_b_pin  = 6
    right_a_pin = 16   right_b_pin = 26
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class QuadratureChannel:
    """Software x4 quadrature decoder for one wheel.

    Maintains a signed cumulative tick count. `on_edge` is called from GPIO
    edge callbacks (any edge on A or B). The classic state-transition table
    is used so that missed/ботh edges still decode correctly.
    """

    # Transition table indexed by (prev_state << 2 | curr_state) -> delta.
    # state = (A << 1) | B. Valid single-step transitions map to +/-1,
    # invalid (double/missed) transitions map to 0.
    _TABLE = (
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0,
    )

    def __init__(self):
        self.ticks = 0
        self._state = 0

    def init_state(self, a_level, b_level):
        self._state = (a_level << 1) | b_level

    def update(self, a_level, b_level):
        curr = (a_level << 1) | b_level
        delta = self._TABLE[(self._state << 2) | curr]
        self.ticks += delta
        self._state = curr
        return delta


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        # --- geometry / encoder parameters ---
        self.declare_parameter('dry_run', False)
        self.declare_parameter('left_a_pin_bcm', 5)
        self.declare_parameter('left_b_pin_bcm', 6)
        self.declare_parameter('right_a_pin_bcm', 16)
        self.declare_parameter('right_b_pin_bcm', 26)
        self.declare_parameter('encoder_cpr', 11)          # counts per motor rev (single channel)
        self.declare_parameter('gear_ratio', 90.0)          # motor:wheel reduction
        self.declare_parameter('wheel_radius', 0.0325)      # m
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('publish_rate', 50.0)        # Hz for speed topic
        self.declare_parameter('speed_filter_alpha', 0.3)   # 0..1 low-pass on wheel speed
        self.declare_parameter('ticks_topic', 'wheel_ticks_std')
        self.declare_parameter('speeds_topic', 'wheel_speeds_std')

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.cpr = int(self.get_parameter('encoder_cpr').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)
        self.alpha = float(self.get_parameter('speed_filter_alpha').value)

        # x4 decode -> 4 * CPR counts per motor rev, times gearbox
        self.ticks_per_wheel_rev = 4.0 * self.cpr * self.gear_ratio
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_wheel_rev
        self.rad_per_tick = (2.0 * math.pi) / self.ticks_per_wheel_rev

        self.left = QuadratureChannel()
        self.right = QuadratureChannel()

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = time.monotonic()
        self.v_left_f = 0.0
        self.v_right_f = 0.0

        self.ticks_pub = self.create_publisher(
            Int32MultiArray, str(self.get_parameter('ticks_topic').value), 10)
        self.speeds_pub = self.create_publisher(
            Float32MultiArray, str(self.get_parameter('speeds_topic').value), 10)

        self._setup_gpio()

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self.publish_loop)
        self.get_logger().info(
            'encoder_node ready: ticks/wheel_rev=%.1f, m/tick=%.6f, dry_run=%s'
            % (self.ticks_per_wheel_rev, self.meters_per_tick, self.dry_run))

    # ------------------------------------------------------------------ GPIO
    def _setup_gpio(self):
        self._backend = 'dry_run'
        if self.dry_run:
            self.get_logger().warn('dry_run enabled: encoders generate no ticks')
            return

        la = int(self.get_parameter('left_a_pin_bcm').value)
        lb = int(self.get_parameter('left_b_pin_bcm').value)
        ra = int(self.get_parameter('right_a_pin_bcm').value)
        rb = int(self.get_parameter('right_b_pin_bcm').value)
        self._pins = {'la': la, 'lb': lb, 'ra': ra, 'rb': rb}

        try:
            import lgpio
            self._lgpio = lgpio
            self._chip = lgpio.gpiochip_open(0)
            for p in (la, lb, ra, rb):
                lgpio.gpio_claim_alert(self._chip, p, lgpio.BOTH_EDGES)
            self.left.init_state(lgpio.gpio_read(self._chip, la),
                                 lgpio.gpio_read(self._chip, lb))
            self.right.init_state(lgpio.gpio_read(self._chip, ra),
                                  lgpio.gpio_read(self._chip, rb))
            self._cb = lgpio.callback(self._chip, la, lgpio.BOTH_EDGES, self._lg_cb)
            lgpio.callback(self._chip, lb, lgpio.BOTH_EDGES, self._lg_cb)
            lgpio.callback(self._chip, ra, lgpio.BOTH_EDGES, self._lg_cb)
            lgpio.callback(self._chip, rb, lgpio.BOTH_EDGES, self._lg_cb)
            self._backend = 'lgpio'
            self.get_logger().info('encoder backend: lgpio')
            return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('lgpio unavailable (%s), trying RPi.GPIO' % exc)

        try:
            import RPi.GPIO as GPIO
            self._GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            for p in (la, lb, ra, rb):
                GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.left.init_state(GPIO.input(la), GPIO.input(lb))
            self.right.init_state(GPIO.input(ra), GPIO.input(rb))
            GPIO.add_event_detect(la, GPIO.BOTH, callback=self._rpi_cb_left)
            GPIO.add_event_detect(lb, GPIO.BOTH, callback=self._rpi_cb_left)
            GPIO.add_event_detect(ra, GPIO.BOTH, callback=self._rpi_cb_right)
            GPIO.add_event_detect(rb, GPIO.BOTH, callback=self._rpi_cb_right)
            self._backend = 'RPi.GPIO'
            self.get_logger().info('encoder backend: RPi.GPIO')
            return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('RPi.GPIO unavailable (%s), falling back to dry_run' % exc)
            self.dry_run = True

    def _lg_cb(self, chip, gpio, level, tick):
        # any edge -> re-read both channels of the affected wheel and decode
        g = self._lgpio
        p = self._pins
        if gpio in (p['la'], p['lb']):
            self.left.update(g.gpio_read(self._chip, p['la']),
                             g.gpio_read(self._chip, p['lb']))
        else:
            self.right.update(g.gpio_read(self._chip, p['ra']),
                              g.gpio_read(self._chip, p['rb']))

    def _rpi_cb_left(self, channel):
        g = self._GPIO
        p = self._pins
        self.left.update(g.input(p['la']), g.input(p['lb']))

    def _rpi_cb_right(self, channel):
        g = self._GPIO
        p = self._pins
        self.right.update(g.input(p['ra']), g.input(p['rb']))

    # ------------------------------------------------------------- publishing
    def publish_loop(self):
        now = time.monotonic()
        dt = now - self.prev_time
        if dt <= 1e-6:
            return
        self.prev_time = now

        lt = self.left.ticks * (-1 if self.invert_left else 1)
        rt = self.right.ticks * (-1 if self.invert_right else 1)

        dl = lt - self.prev_left_ticks
        dr = rt - self.prev_right_ticks
        self.prev_left_ticks = lt
        self.prev_right_ticks = rt

        v_left = (dl * self.meters_per_tick) / dt
        v_right = (dr * self.meters_per_tick) / dt

        # low-pass filter (encoders on cheap motors are noisy at 50 Hz)
        self.v_left_f += self.alpha * (v_left - self.v_left_f)
        self.v_right_f += self.alpha * (v_right - self.v_right_f)

        ticks_msg = Int32MultiArray()
        ticks_msg.data = [int(lt), int(rt)]
        self.ticks_pub.publish(ticks_msg)

        speeds_msg = Float32MultiArray()
        speeds_msg.data = [float(self.v_left_f), float(self.v_right_f)]
        self.speeds_pub.publish(speeds_msg)

    def destroy_node(self):
        try:
            if self._backend == 'lgpio':
                self._lgpio.gpiochip_close(self._chip)
            elif self._backend == 'RPi.GPIO':
                self._GPIO.cleanup()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
