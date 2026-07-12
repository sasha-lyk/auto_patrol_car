import os
import sys
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'raspi_car_base'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'raspi_car_bringup'))

from raspi_car_base.control_core import DifferentialOdometry, wheel_targets  # noqa: E402
from raspi_car_bringup.route_core import (  # noqa: E402
    RouteValidationError, map_bundle_sha256, validate_route)
from raspi_car_bringup.safety_core import LatchedEmergencyStop, VelocityArbiter  # noqa: E402


class CoreTests(unittest.TestCase):
    def test_wheel_targets(self):
        left, right = wheel_targets(0.2, 1.0, 0.2)
        self.assertAlmostEqual(left, 0.1)
        self.assertAlmostEqual(right, 0.3)

    def test_exact_straight_odometry(self):
        odom = DifferentialOdometry(0.2)
        odom.update(1.0, 1.0)
        self.assertAlmostEqual(odom.x, 1.0)
        self.assertAlmostEqual(odom.y, 0.0)
        self.assertAlmostEqual(odom.theta, 0.0)

    def test_estop_is_latched(self):
        estop = LatchedEmergencyStop()
        estop.trigger(now=1.0)
        self.assertTrue(estop.latched)
        self.assertTrue(estop.latched)
        estop.reset(now=999.0)
        self.assertFalse(estop.latched)

    def test_arbiter_timeout_and_priority(self):
        arbiter = VelocityArbiter({
            'web': {'priority': 100, 'timeout': 0.8},
            'nav': {'priority': 50, 'timeout': 0.6},
        })
        arbiter.update('nav', 'nav', now=0.0)
        arbiter.update('web', 'web', now=0.1)
        self.assertEqual(arbiter.select(now=0.2), ('web', 'web'))
        self.assertEqual(arbiter.select(now=1.0), ('', None))

    def test_uncalibrated_route_is_rejected(self):
        data = {
            'route': {'calibrated': False, 'frame_id': 'map'},
            'waypoints': [
                {'x': 0, 'y': 0, 'yaw': 0}, {'x': 1, 'y': 0, 'yaw': 0}],
        }
        with self.assertRaises(RouteValidationError):
            validate_route(data)

    def test_waypoint_outside_map_is_rejected(self):
        map_yaml = os.path.join(
            ROOT, 'src', 'raspi_car_bringup', 'maps', 'room1.yaml')
        data = {
            'route': {
                'calibrated': True, 'frame_id': 'map', 'clearance_m': 0.0,
                'map_sha256': map_bundle_sha256(map_yaml),
            },
            'waypoints': [
                {'x': 1000, 'y': 1000, 'yaw': 0},
                {'x': 1001, 'y': 1000, 'yaw': 0},
            ],
        }
        with self.assertRaises(RouteValidationError):
            validate_route(data, map_yaml=map_yaml)


if __name__ == '__main__':
    unittest.main()
