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


class SerialIMURPMReader(Node):
    def __init__(self):
        super().__init__('serial_imu_rpm_reader')
        
        # Subscripción a los datos combinados de IMU y VESC
        self.imu_rpm_sub = self.create_subscription(
            Float32MultiArray,
            '/imu_data',
            self.imu_rpm_callback,
            10
        )
        
        # Subscripción al punto clickeado
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, 10
        )

        # Publicadores de la IMU
        self.publisher_yaw = self.create_publisher(Float32, 'imu/yaw', 10)
        self.publisher_std_yaw = self.create_publisher(Float32, 'imu/yaw_std', 10)
        self.publisher_accelx = self.create_publisher(Float32, 'imu/accel_x', 10)
        self.publisher_accely = self.create_publisher(Float32, 'imu/accel_y', 10)
        self.publisher_accelz = self.create_publisher(Float32, 'imu/accel_z', 10)
        self.publisher_gyrox = self.create_publisher(Float32, 'imu/gyro_x', 10)
        self.publisher_gyroy = self.create_publisher(Float32, 'imu/gyro_y', 10)
        self.publisher_gyroz = self.create_publisher(Float32, 'imu/gyro_z', 10)

        # Publisher del punto procesado
        self.clicked_point_pub = self.create_publisher(PointStamped, '/clicked_point_processed', 10)

        # Publicadores de velocidad lineal y su desviación estándar
        self.linear_vel_pub = self.create_publisher(Float32, '/linear_velocity', 10)
        self.std_linear_vel_pub = self.create_publisher(Float32, '/linear_velocity/stddev', 10)

        # Estado
        self.alpha = 0.1
        self.ema_means = [0.0] * 4
        self.ema_vars = [0.0] * 4
        self.wheel_radius = 0.12  # metros

    def update_ema_std(self, new_rpms):
        stds = []
        for i in range(4):
            delta = new_rpms[i] - self.ema_means[i]
            self.ema_means[i] += self.alpha * delta
            self.ema_vars[i] = (1 - self.alpha) * (self.ema_vars[i] + self.alpha * delta ** 2)
            stds.append(math.sqrt(self.ema_vars[i]))
        return stds

    def imu_rpm_callback(self, msg):
        try:
            data = msg.data
            if len(data) < 14:
                self.get_logger().warn("Mensaje /imu_data demasiado corto.")
                return

            # Desempaquetar datos (ajustar según el orden real del array)
            yaw = data[0]
            yaw_std = data[1]
            accelx, accely, accelz = data[2], data[3], data[4]
            gyrox, gyroy, gyroz = data[5], data[6], data[7]
            rpms = data[8:12]  # RPMs de las 4 ruedas

            # Publicar IMU
            self.publisher_yaw.publish(Float32(data=yaw))
            self.publisher_std_yaw.publish(Float32(data=yaw_std))
            self.publisher_accelx.publish(Float32(data=accelx))
            self.publisher_accely.publish(Float32(data=accely))
            self.publisher_accelz.publish(Float32(data=accelz))
            self.publisher_gyrox.publish(Float32(data=gyrox))
            self.publisher_gyroy.publish(Float32(data=gyroy))
            self.publisher_gyroz.publish(Float32(data=gyroz))

            # Calcular velocidad lineal y su desviación estándar
            rpm_stds_smoothed = self.update_ema_std(rpms)
            avg_rpm = sum(self.ema_means) / 4.0
            linear_velocity = (2 * math.pi * self.wheel_radius / 60.0) * avg_rpm
            std_linear_vel = (2 * math.pi * self.wheel_radius / 60.0) * (
                math.sqrt(sum(var for var in self.ema_vars)) / 4.0
            )

            self.linear_vel_pub.publish(Float32(data=linear_velocity))
            self.std_linear_vel_pub.publish(Float32(data=std_linear_vel))

        except Exception as e:
            self.get_logger().warn(f"Error en imu_rpm_callback: {e}")

    def clicked_point_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        self.get_logger().info(f'Recibido en /clicked_point → x: {x:.2f}, y: {y:.2f}, z: {z:.2f}')
        self.clicked_point_pub.publish(msg)
