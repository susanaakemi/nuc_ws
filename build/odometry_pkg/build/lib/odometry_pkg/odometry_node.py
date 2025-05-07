#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, PointStamped
from std_msgs.msg import Float32, Float32MultiArray
from odometry_pkg.controller import angulo_ackermann, find_look_ahead_point, generar_ruta_prioritaria, find_stopping_point, robot_stop
from odometry_pkg.efk import compute_F, predict_state
from odometry_pkg.utils import compute_quaternion
from odometry_pkg.utils import ListQueueSimple
import time
import serial


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.wheel_pub = self.create_publisher(Twist, '/wheel_setter', 10)
        self.rpm_publisher = self.create_publisher(Float32MultiArray, 'target_rpms', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.start_time = time.time()
        
        self.dt = 0.05
        self.lookAheadDist = 1.5
        self.desiredSpeed = 0.4
        self.L = 0.89
        self.wheelDiameter = 0.24
        self.wheelCircumference = np.pi * self.wheelDiameter

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = np.radians(0.0)
        self.xhat = np.array([self.odom_x, self.odom_y, self.odom_theta, 0.0, 0.0, 0.0])
        self.P = np.identity(6) * 0.01
        self.Q = np.diag([0.001, 0.001, 0.0005, 0.001, 0.001, 0.0001])
        self.R = np.diag([0.02, 0.02, 0.01, 0.01, 0.01])

        self.idxWaypoint = 0
        self.waypoints = ListQueueSimple()  # Cambiado a ListQueueSimple

        # Suscripciones
        self.create_subscription(Float32, '/linear_velocity', self.linear_velocity_callback, 10)
        self.create_subscription(Float32, '/linear_velocity_stddev', self.linear_velocity_stddev_callback, 10)
        self.create_subscription(Float32, '/orientation_z', self.orientation_z_callback, 10)
        self.create_subscription(Float32, '/linear_acceleration_x', self.linear_accel_x_callback, 10)
        self.create_subscription(Float32, '/angular_velocity_z', self.angular_velocity_z_callback, 10)
        self.create_subscription(Float32, '/angular_velocity_y', self.angular_velocity_y_callback, 10)
        self.create_subscription(PointStamped, '/clicked_point_processed', self.clicked_point_callback, 10)
        self.create_subscription(Float32, '/linear_acceleration_x_stddev', self.linear_accel_x_stddev_callback, 10)

        # Inicialización de variables
        self.real_velocity = 0.0
        self.prev_ema_std_v = 0.0
        self.orientation_z = 0.0
        self.linear_acceleration_x = 0.0
        self.angular_velocity_z = 0.0
        self.linear_acceleration_x_stddev = 0.0
        self.angular_velocity_y = 0.0
        self.last_clicked_point = Point()

        # Iniciar comunicación serial
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.get_logger().info("Serial port initialized")

    def timer_callback(self):
        # Asegurarse de que la cola no esté vacía antes de procesar
        if self.waypoints.size() == 0:
            self.get_logger().warn("La cola de waypoints está vacía. No se procesarán más puntos.")
            return

        # Ajuste de la matriz de covarianza R
        self.R = np.diag([
            0.02, 0.02, 0.01,
            self.prev_ema_std_v + self.linear_acceleration_x_stddev,
            self.orientation_z
        ])

        # Obtención de la última posición del punto clickeado
        piedra_x = self.last_clicked_point.x
        piedra_dist = self.last_clicked_point.y

        # Calcular punto de parada
        stopping_point = find_stopping_point(piedra_x, piedra_dist, self.xhat[0], self.xhat[1], self.xhat[2])
        if robot_stop(stopping_point, self.odom_x, self.odom_y):
            self.send_rpm([0, 0, 0, 0])
            self.get_logger().info("Robot detenido en el punto de parada.")
            return

        # Generar la ruta prioritaria
        next_point = generar_ruta_prioritaria(stopping_point, use_push_front=False)
        if next_point:
            self.waypoints.enqueue(next_point)  # Usar enqueue para agregar puntos a la cola
            if self.waypoints.size() > 100:  # Verificar el tamaño de la cola usando size()
                for _ in range(self.waypoints.size() - 50):
                    self.waypoints.dequeue()  # Mantener solo los 50 puntos más recientes

        # Procesar los waypoints
        waypoints_array = np.array(self.waypoints.items)  # Convertir los items de la cola a un array
        lookX, lookY, self.idxWaypoint = find_look_ahead_point(
            self.odom_x, self.odom_y, waypoints_array, self.idxWaypoint, self.lookAheadDist)

        if self.idxWaypoint > 0:
            self.waypoints.items = self.waypoints.items[self.idxWaypoint:]  # Ajustar la cola eliminando los puntos ya procesados
            self.idxWaypoint = 0

        # Calcular el control Ackermann para los ángulos y las velocidades de las ruedas
        dx = lookX - self.odom_x
        dy = lookY - self.odom_y
        L_d = np.hypot(dx, dy)
        alpha = np.arctan2(dy, dx) - self.odom_theta
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        kappa = 2 * np.sin(alpha) / max(L_d, 1e-9)
        delta = np.arctan(kappa * self.L)
        b, v_interior = angulo_ackermann(delta, self.desiredSpeed)

        rpm_exterior = (self.desiredSpeed / self.wheelCircumference) * 60
        rpm_interior = (v_interior / self.wheelCircumference) * 60

        if delta > 0:
            rpm_left = rpm_interior
            rpm_right = rpm_exterior
        elif delta < 0:
            rpm_left = rpm_exterior
            rpm_right = rpm_interior
        else:
            rpm_left = rpm_right = rpm_exterior

        self.send_rpm([rpm_left, rpm_right, rpm_left, rpm_right])
        self.send_angles([delta, b, -delta, -b])

        # Actualización de la odometría
        z = np.array([
            self.odom_x,
            self.odom_y,
            self.odom_theta,
            self.real_velocity + self.linear_acceleration_x,
            self.angular_velocity_z
        ]) + np.random.normal(0, np.sqrt(np.diag(self.R)))

        omega = (self.real_velocity / self.L) * np.tan(delta)
        self.odom_x += self.real_velocity * np.cos(self.odom_theta) * self.dt
        self.odom_y += self.real_velocity * np.sin(self.odom_theta) * self.dt
        self.odom_theta += omega * self.dt

        u = np.array([self.real_velocity, delta])
        sensor_data = {
            'linear_acceleration': {'x': self.linear_acceleration_x},
            'angular_velocity': {'z': self.angular_velocity_z}
        }

        # Predicción del estado usando el filtro de Kalman
        xhat_pred = predict_state(self.xhat, u, sensor_data, self.L, self.dt)
        F = compute_F(self.xhat, u, sensor_data, self.dt)

        P_pred = F @ self.P @ F.T + self.Q
        H = np.eye(6)[:5]
        S = H @ P_pred @ H.T + self.R
        K = P_pred @ H.T @ np.linalg.inv(S)
        self.xhat = xhat_pred + K @ (z - H @ xhat_pred)
        self.P = (np.eye(6) - K @ H) @ P_pred

        # Publicar la odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.xhat[0]
        odom_msg.pose.pose.position.y = self.xhat[1]
        odom_msg.pose.pose.position.z = 0

        quaternion = compute_quaternion(self.xhat[2])
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        wheel_msg = Twist()
        wheel_msg.linear.x = v_interior
        wheel_msg.linear.y = self.desiredSpeed
        wheel_msg.angular.x = delta
        wheel_msg.angular.y = b

        self.odom_pub.publish(odom_msg)
        self.wheel_pub.publish(wheel_msg)

    def destroy_node(self):
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

