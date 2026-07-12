"""YB_MRA02 9-axis IMU UART driver (unchanged from the working base version).

Publishes sensor_msgs/Imu and sensor_msgs/MagneticField. The orientation and
angular_velocity are what robot_localization's EKF fuses with wheel odometry
to keep heading (yaw) drift-free.
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField


FRAME_HEAD1 = 0x7E
FRAME_HEAD2 = 0x23
FUNC_RAW_ACCEL = 0x04
FUNC_QUAT = 0x16
FUNC_EULER = 0x26
FUNC_REQUEST_DATA = 0x80


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


class YbMra02Parser:
    def __init__(self):
        self.state = 0
        self.length = 0
        self.function = 0
        self.buffer = bytearray()

    def push(self, byte_value):
        if self.state == 0:
            self.state = 1 if byte_value == FRAME_HEAD1 else 0
            return None
        if self.state == 1:
            self.state = 2 if byte_value == FRAME_HEAD2 else 0
            return None
        if self.state == 2:
            self.length = byte_value
            self.state = 3
            return None
        if self.state == 3:
            self.function = byte_value
            self.buffer = bytearray()
            data_length = self.length - 4
            self.state = 4 if 0 < data_length <= 64 else 0
            return None
        if self.state == 4:
            self.buffer.append(byte_value)
            data_length = self.length - 4
            if len(self.buffer) < data_length:
                return None
            payload = self.buffer[:-1]
            received_checksum = self.buffer[-1]
            checksum = (FRAME_HEAD1 + FRAME_HEAD2 + self.length
                        + self.function + sum(payload)) & 0xFF
            frame = None
            if checksum == received_checksum:
                frame = (self.function, bytes(payload))
            self.state = 0
            return frame
        self.state = 0
        return None


class YbMra02UartNode(Node):
    def __init__(self):
        super().__init__('yb_mra02_uart_node')
        self.declare_parameter('serial_port', '/dev/serial0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('mag_topic', 'mag')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('request_data', True)
        self.declare_parameter('request_rate', 20.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.request_data = bool(self.get_parameter('request_data').value)
        self.parser = YbMra02Parser()

        self.accel = [0.0, 0.0, 9.80665]
        self.gyro = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        self.quat = None
        self.euler = None

        import serial
        self.serial = serial.Serial(
            str(self.get_parameter('serial_port').value),
            int(self.get_parameter('baud_rate').value), timeout=0.0)

        self.imu_pub = self.create_publisher(Imu, str(self.get_parameter('imu_topic').value), 10)
        self.mag_pub = self.create_publisher(MagneticField, str(self.get_parameter('mag_topic').value), 10)

        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value), self.poll_and_publish)
        rr = float(self.get_parameter('request_rate').value)
        if self.request_data and rr > 0:
            self.create_timer(1.0 / rr, self.request_sensor_data)
        self.get_logger().info('YB_MRA02 UART ready: port=%s' % self.serial.port)

    def request_sensor_data(self):
        self.send_command(FUNC_REQUEST_DATA, bytes([FUNC_RAW_ACCEL, 0x00]))
        self.send_command(FUNC_REQUEST_DATA, bytes([FUNC_QUAT, 0x00]))
        self.send_command(FUNC_REQUEST_DATA, bytes([FUNC_EULER, 0x00]))

    def send_command(self, function, payload):
        frame_length = 4 + len(payload) + 1
        frame = bytearray([FRAME_HEAD1, FRAME_HEAD2, frame_length, function])
        frame.extend(payload)
        frame.append(sum(frame) & 0xFF)
        self.serial.write(frame)

    def poll_and_publish(self):
        waiting = self.serial.in_waiting
        if waiting:
            for byte_value in self.serial.read(waiting):
                frame = self.parser.push(byte_value)
                if frame is not None:
                    self.parse_frame(*frame)
        self.publish_imu()
        self.publish_mag()

    def parse_frame(self, function, payload):
        if function == FUNC_RAW_ACCEL and len(payload) >= 18:
            raw = struct.unpack_from('<hhhhhhhhh', payload)
            accel_ratio = 16.0 / 32767.0 * 9.80665
            gyro_ratio = 2000.0 / 32767.0 * math.pi / 180.0
            mag_ratio = 800.0 / 32767.0 * 1e-6
            self.accel = [raw[0] * accel_ratio, raw[1] * accel_ratio, raw[2] * accel_ratio]
            self.gyro = [raw[3] * gyro_ratio, raw[4] * gyro_ratio, raw[5] * gyro_ratio]
            self.mag = [raw[6] * mag_ratio, raw[7] * mag_ratio, raw[8] * mag_ratio]
        elif function == FUNC_QUAT and len(payload) >= 16:
            q0, q1, q2, q3 = struct.unpack_from('<ffff', payload)
            norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
            if norm > 1e-6:
                self.quat = (q0 / norm, q1 / norm, q2 / norm, q3 / norm)
        elif function == FUNC_EULER and len(payload) >= 12:
            roll, pitch, yaw = struct.unpack_from('<fff', payload)
            if max(abs(roll), abs(pitch), abs(yaw)) > math.tau:
                roll = math.radians(roll); pitch = math.radians(pitch); yaw = math.radians(yaw)
            self.euler = (roll, pitch, yaw)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        orientation = self.quat
        if orientation is None and self.euler is not None:
            orientation = quaternion_from_euler(*self.euler)
        if orientation is not None:
            msg.orientation.w = orientation[0]
            msg.orientation.x = orientation[1]
            msg.orientation.y = orientation[2]
            msg.orientation.z = orientation[3]
            msg.orientation_covariance[0] = 0.05
            msg.orientation_covariance[4] = 0.05
            msg.orientation_covariance[8] = 0.10
        else:
            msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = self.gyro[0]
        msg.angular_velocity.y = self.gyro[1]
        msg.angular_velocity.z = self.gyro[2]
        msg.linear_acceleration.x = self.accel[0]
        msg.linear_acceleration.y = self.accel[1]
        msg.linear_acceleration.z = self.accel[2]
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02
        msg.linear_acceleration_covariance[0] = 0.2
        msg.linear_acceleration_covariance[4] = 0.2
        msg.linear_acceleration_covariance[8] = 0.2
        self.imu_pub.publish(msg)

    def publish_mag(self):
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.magnetic_field.x = self.mag[0]
        msg.magnetic_field.y = self.mag[1]
        msg.magnetic_field.z = self.mag[2]
        self.mag_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'serial') and self.serial is not None:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YbMra02UartNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
