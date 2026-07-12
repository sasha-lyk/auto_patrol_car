"""ROS-independent control and differential-drive math."""

import math


class Pid:
    def __init__(self, kp, ki, kd, out_min=-1.0, out_max=1.0, i_max=0.5):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_min = float(out_min)
        self.out_max = float(out_max)
        self.i_max = float(i_max)
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def step(self, target, measured, dt):
        if dt <= 1e-6:
            return 0.0
        error = target - measured
        self.integral = max(
            -self.i_max, min(self.i_max, self.integral + error * dt))
        derivative = (error - self.prev_err) / dt
        self.prev_err = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, output))


def wheel_targets(linear_x, angular_z, wheel_base):
    half = float(wheel_base) * 0.5
    return linear_x - angular_z * half, linear_x + angular_z * half


class DifferentialOdometry:
    def __init__(self, wheel_separation):
        self.wheel_separation = float(wheel_separation)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, d_left, d_right):
        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.wheel_separation
        if abs(d_theta) < 1e-6:
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
        else:
            radius = d_center / d_theta
            self.x += radius * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + d_theta) - math.cos(self.theta))
        self.theta = math.atan2(
            math.sin(self.theta + d_theta), math.cos(self.theta + d_theta))
        return self.x, self.y, self.theta, d_center, d_theta
