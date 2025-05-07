#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu
import math
from threading import Lock
import transforms3d.quaternions as quat


class ListQueueSimple:
    """A thread-safe queue for storing items with FIFO behavior."""

    def __init__(self):
        self.items = []
        self.lock = Lock()

    def enqueue(self, item):
        with self.lock:
            self.items.append(item)

    def push_front(self, item):
        with self.lock:
            self.items.insert(0, item)

    def dequeue(self):
        with self.lock:
            if self.items:
                return self.items.pop(0)
            return None

    """def isempty(self):
        with self.lock:
            return len(self.items) == 0"""

    def size(self):
        with self.lock:
            return len(self.items)


def compute_quaternion(theta):
    q = quat.axangle2quat([0, 0, 1], theta)
    return [q[1], q[2], q[3], q[0]]


class IMU_VESCListener(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.imu_data = {
            'stamp': None,
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        self.rpms = {'stamp': None, 'data': [0.0] * 4}
        self.stddev = {'stamp': None, 'data': [0.0] * 13}

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.rpm_sub = self.create_subscription(Float32MultiArray, '/vesc_rpms', self.rpm_callback, 10)
        self.stddev_sub = self.create_subscription(Float32MultiArray, '/imu_rpm_stddev', self.stddev_callback, 10)
        self.clicked_point_sub = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)

        # Publishers
        self.rpm_publisher = self.create_publisher(Float32MultiArray, 'wheel_rpm_cmd', 10)
        self.linear_velocity_pub = self.create_publisher(Float32, '/linear_velocity', 10)
        self.linear_velocity_stddev_pub = self.create_publisher(Float32, '/linear_velocity_stddev', 10)
        self.orientation_z_pub = self.create_publisher(Float32, '/orientation_z', 10)
        self.linear_accel_x_pub = self.create_publisher(Float32, '/linear_acceleration_x', 10)
        self.angular_velocity_z_pub = self.create_publisher(Float32, '/angular_velocity_z', 10)
        self.angular_velocity_y_pub = self.create_publisher(Float32, '/angular_velocity_y', 10)
        self.linear_acc_x_stddev_pub = self.create_publisher(Float32, '/linear_acceleration_x_stddev', 10)
        self.clicked_point_pub = self.create_publisher(PointStamped, '/clicked_point_processed', 10)

        self.wheel_radius = 0.12
        self.ema_alpha = 0.1
        self.prev_ema_std_v = 0.0

    def clicked_point_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        self.get_logger().info(f'Recibido en /clicked_point → x: {x:.2f}, y: {y:.2f}, z: {z:.2f}')
        self.clicked_point_pub.publish(msg)

    def calculate_velocity_metrics(self, rpms, rpm_stddevs):
        linear_velocities = [(rpm * 2 * math.pi / 60) * self.wheel_radius for rpm in rpms]
        self.linear_velocity = sum(linear_velocities) / len(linear_velocities)

        linear_vel_stddevs = [(std * 2 * math.pi / 60) * self.wheel_radius for std in rpm_stddevs]
        variance_sum = sum(std ** 2 for std in linear_vel_stddevs) / len(linear_vel_stddevs)
        system_std_v = math.sqrt(variance_sum)

        ema_std_v = self.ema_alpha * system_std_v + (1 - self.ema_alpha) * self.prev_ema_std_v
        self.prev_ema_std_v = ema_std_v

        vel_msg = Float32()
        vel_msg.data = self.linear_velocity
        self.linear_velocity_pub.publish(vel_msg)

        stddev_msg = Float32()
        stddev_msg.data = self.prev_ema_std_v
        self.linear_velocity_stddev_pub.publish(stddev_msg)

        return self.linear_velocity, ema_std_v

    def imu_callback(self, msg):
        self.imu_data['stamp'] = msg.header.stamp
        self.imu_data['orientation']['x'] = msg.orientation.x
        self.imu_data['orientation']['y'] = msg.orientation.y
        self.imu_data['orientation']['z'] = msg.orientation.z
        self.imu_data['linear_acceleration']['x'] = msg.linear_acceleration.x
        self.imu_data['linear_acceleration']['y'] = msg.linear_acceleration.y
        self.imu_data['linear_acceleration']['z'] = msg.linear_acceleration.z
        self.imu_data['angular_velocity']['x'] = msg.angular_velocity.x
        self.imu_data['angular_velocity']['y'] = msg.angular_velocity.y
        self.imu_data['angular_velocity']['z'] = msg.angular_velocity.z

        orientation_z_msg = Float32()
        orientation_z_msg.data = self.imu_data['orientation']['z']
        self.orientation_z_pub.publish(orientation_z_msg)

        linear_accel_x_msg = Float32()
        linear_accel_x_msg.data = self.imu_data['linear_acceleration']['x']
        self.linear_accel_x_pub.publish(linear_accel_x_msg)

        angular_vel_z_msg = Float32()
        angular_vel_z_msg.data = self.imu_data['angular_velocity']['z']
        self.angular_velocity_z_pub.publish(angular_vel_z_msg)

        angular_vel_y_msg = Float32()
        angular_vel_y_msg.data = self.imu_data['angular_velocity']['y']
        self.angular_velocity_y_pub.publish(angular_vel_y_msg)

    def rpm_callback(self, msg):
        self.rpms['stamp'] = self.imu_data['stamp']
        self.rpms['data'] = list(msg.data)

    def stddev_callback(self, msg):
        self.stddev['stamp'] = self.imu_data['stamp']
        self.stddev['data'] = list(msg.data)

        if self.rpms['stamp'] is None or self.imu_data['stamp'] is None:
            self.get_logger().warn("RPM or IMU data not available yet")
            return

        rpms = self.rpms['data']
        rpm_stddevs = self.stddev['data'][9:13]
        timestamp = self.imu_data['stamp'].sec + self.imu_data['stamp'].nanosec / 1e9

        linear_velocity, linear_velocity_stddev = self.calculate_velocity_metrics(rpms, rpm_stddevs)

        self.get_logger().info(
            f"Velocity Data: Time={timestamp:.3f}, "
            f"Linear Vel={linear_velocity:.2f} m/s, "
            f"Linear Vel Stddev={linear_velocity_stddev:.2f} m/s"
        )

        self.send_rpm(rpms)

        std_acc_x = self.stddev['data'][0]
        stddev_msg = Float32()
        stddev_msg.data = std_acc_x
        self.linear_acc_x_stddev_pub.publish(stddev_msg)

    def send_rpm(self, rpms):
        msg = Float32MultiArray()
        msg.data = rpms
        self.rpm_publisher.publish(msg)

    def send_angles(self, angles):
        msg = Float32MultiArray()
        msg.data = angles
        self.rpm_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMU_VESCListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

