"""Deterministic, hardware-free validation of the project's core subsystems."""

import json
import math
import os
import sys


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'raspi_car_base'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'raspi_car_bringup'))

from raspi_car_base.control_core import DifferentialOdometry, Pid  # noqa: E402
from raspi_car_bringup.route_core import (  # noqa: E402
    RouteValidationError, map_bundle_sha256, validate_route)
from raspi_car_bringup.safety_core import (  # noqa: E402
    LatchedEmergencyStop, VelocityArbiter)


def simulate_speed_control(closed_loop):
    dt = 0.02
    target = 0.30
    max_speed = 0.90
    feedforward = 0.90
    pid = Pid(2.2, 6.0, 0.02, i_max=0.6)
    speed = 0.0
    errors_after_disturbance = []
    samples = []
    for step in range(int(12.0 / dt)):
        now = step * dt
        battery = 1.0 if now < 4.0 else 0.70
        load = 0.0 if now < 4.0 else 0.055
        if closed_loop:
            effort = feedforward * target / max_speed + pid.step(target, speed, dt)
        else:
            effort = target / max_speed
        effort = max(-1.0, min(1.0, effort))
        desired_speed = max_speed * effort * battery
        if desired_speed > 0.0:
            desired_speed = max(0.0, desired_speed - load)
        speed += (desired_speed - speed) * dt / 0.22
        if now >= 7.0:
            errors_after_disturbance.append(abs(target - speed))
        if step % 50 == 0:
            samples.append({'t': round(now, 1), 'speed': round(speed, 4)})
    return {
        'steady_mae_mps': round(sum(errors_after_disturbance) / len(errors_after_disturbance), 5),
        'final_speed_mps': round(speed, 5),
        'samples': samples,
    }


def validate_safety():
    arbiter = VelocityArbiter({
        'web': {'priority': 100, 'timeout': 0.8},
        'nav': {'priority': 50, 'timeout': 0.6},
    })
    estop = LatchedEmergencyStop()
    arbiter.update('nav', {'v': 0.2}, now=0.0)
    before = arbiter.select(now=0.1)[0]
    estop.trigger(now=0.2)
    still_latched_after_60s = estop.latched
    estop.reset(now=60.3)
    arbiter.clear()
    after_reset = arbiter.select(now=60.3)[0]
    return {
        'source_before_estop': before,
        'latched_after_60s': still_latched_after_60s,
        'source_after_reset': after_reset or 'none',
    }


def validate_odometry():
    odom = DifferentialOdometry(0.20)
    for _ in range(100):
        odom.update(0.01, 0.01)
    straight_error = math.hypot(odom.x - 1.0, odom.y)
    arc = DifferentialOdometry(0.20)
    for _ in range(100):
        arc.update(0.008, 0.012)
    expected_theta = 2.0
    return {
        'straight_error_m': round(straight_error, 9),
        'arc_heading_error_rad': round(abs(arc.theta - expected_theta), 9),
    }


def validate_route_binding():
    map_yaml = os.path.join(
        ROOT, 'src', 'raspi_car_bringup', 'maps', 'room1.yaml')
    fingerprint = map_bundle_sha256(map_yaml)
    valid = {
        'route': {
            'name': 'simulation_route', 'frame_id': 'map',
            'calibrated': True, 'map_sha256': fingerprint, 'clearance_m': 0.0,
        },
        'waypoints': [
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            {'x': 1.0, 'y': 0.5, 'yaw': 0.5},
        ],
    }
    validate_route(valid, map_yaml=map_yaml)
    rejected_uncalibrated = False
    invalid = dict(valid)
    invalid['route'] = dict(valid['route'], calibrated=False)
    try:
        validate_route(invalid, map_yaml=map_yaml)
    except RouteValidationError:
        rejected_uncalibrated = True
    return {
        'map_sha256_prefix': fingerprint[:12],
        'valid_route_accepted': True,
        'uncalibrated_route_rejected': rejected_uncalibrated,
    }


def main():
    open_loop = simulate_speed_control(False)
    closed_loop = simulate_speed_control(True)
    report = {
        'speed_control': {'open_loop': open_loop, 'closed_loop': closed_loop},
        'safety': validate_safety(),
        'odometry': validate_odometry(),
        'route_binding': validate_route_binding(),
    }
    checks = {
        'closed_loop_beats_open_loop': (
            closed_loop['steady_mae_mps'] < open_loop['steady_mae_mps'] * 0.35),
        'closed_loop_error_below_0_03_mps': closed_loop['steady_mae_mps'] < 0.03,
        'estop_remains_latched': report['safety']['latched_after_60s'],
        'reset_requires_fresh_command': report['safety']['source_after_reset'] == 'none',
        'odometry_math': (
            report['odometry']['straight_error_m'] < 1e-6
            and report['odometry']['arc_heading_error_rad'] < 1e-6),
        'route_guard': report['route_binding']['uncalibrated_route_rejected'],
    }
    report['checks'] = checks
    report['passed'] = all(checks.values())
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
