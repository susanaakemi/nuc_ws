# utils.py
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Point
from threading import Lock, Thread
import time
import serial
import tf_transformations as tft

class ListQueueSimple:
    """
    A thread-safe queue for storing items (e.g., 2D coordinates) with FIFO behavior.
    Supports enqueue, dequeue, isempty, and push_front for prioritization.
    """
    def __init__(self):
        self.items = []
        self.lock = Lock()

    def enqueue(self, item):
        """Add an item to the tail of the queue."""
        with self.lock:
            self.items.append(item)

    def push_front(self, item):
        """Add an item to the head of the queue (for prioritization)."""
        with self.lock:
            self.items.insert(0, item)

    def dequeue(self):
        """Remove and return an item from the head of the queue."""
        with self.lock:
            if self.items:
                return self.items.pop(0)
            return None

    def isempty(self):
        """Check if the queue is empty."""
        with self.lock:
            return len(self.items) == 0

    def size(self):
        """Return the number of items in the queue."""
        with self.lock:
            return len(self.items)


def compute_quaternion(theta):
    return tft.quaternion_from_euler(0, 0, theta)

class IMU_VESCListener(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        # Storage with timestamps
        self.imu_data = {
            'stamp': None,
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        self.rpms = {'stamp': None, 'data': [0.0] * 4}
        self.stddev = {'stamp': None, 'data': [0.0] * 13}

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu_data', self.imu_callback, 10)
        self.rpm_sub = self.create_subscription(
            Float32MultiArray, '/vesc_rpms', self.rpm_callback, 10)
        self.stddev_sub = self.create_subscription(
            Float32MultiArray, '/imu_rpm_stddev', self.stddev_callback, 10)

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
        stamp_time = self.imu_data['stamp'].sec + self.imu_data['stamp'].nanosec / 1e9
        self.get_logger().info(
            f'IMU Data: Time={stamp_time:.3f}, '
            f'Yaw={self.imu_data["orientation"]["z"]:.2f} deg, '
            f'Accel=[{self.imu_data["linear_acceleration"]["x"]:.2f}, '
            f'{self.imu_data["linear_acceleration"]["y"]:.2f}, '
            f'{self.imu_data["linear_acceleration"]["z"]:.2f}] m/s^2, '
            f'Angular Vel=[{self.imu_data["angular_velocity"]["x"]:.2f}, '
            f'{self.imu_data["angular_velocity"]["y"]:.2f}, '
            f'{self.imu_data["angular_velocity"]["z"]:.2f}] deg/s'
        )

    def rpm_callback(self, msg):
        self.rpms['stamp'] = self.imu_data['stamp']
        self.rpms['data'] = list(msg.data)
        stamp_time = self.rpms['stamp'].sec + self.rpms['stamp'].nanosec / 1e9 if self.rpms['stamp'] else 0.0
        self.get_logger().info(
            f'RPM Data: Time={stamp_time:.3f}, '
            f'RPMs={self.rpms["data"]}'
        )

    def stddev_callback(self, msg):
        self.stddev['stamp'] = self.imu_data['stamp']
        self.stddev['data'] = list(msg.data)
        stamp_time = self.stddev['stamp'].sec + self.stddev['stamp'].nanosec / 1e9 if self.stddev['stamp'] else 0.0
        self.get_logger().info(
            f'Stddev Data: Time={stamp_time:.3f}, '
            f'Std Yaw={self.stddev["data"][6]:.2f} deg, '
            f'Std RPMs={self.stddev["data"][9:13]}'
        )

class CoordinatesListener(Node):
    def __init__(self):
        super().__init__('coordinates_listener')
        self.rock_coords = ListQueueSimple()
        self.create_subscription(Point, "obstacle_coordinates", self.callback, 10)

    def callback(self, msg):
        coord = [msg.x, msg.y]  # Use list for compatibility with waypoints
        self.rock_coords.enqueue(coord)  # Or push_front(coord) for newest-first priority

    def get_new_coords(self):
        return self.rock_coords

class SynchronizedData:
    def __init__(self):
        self.lock = Lock()
        self.last_update_time = 0
        self.imu_data = {}
        self.rpm_data = (0, 0, 0, 0)
        self.linear_velocity = 0.0
        self.linear_velocity_std = 0.0

    def update_imu(self, imu_dict):
        with self.lock:
            self.imu_data = imu_dict
            self.last_update_time = time.time()

    def update_rpm(self, rpm_tuple, linear_vel=0.0, linear_std=0.0):
        with self.lock:
            self.rpm_data = rpm_tuple
            self.linear_velocity = linear_vel
            self.linear_velocity_std = linear_std
            self.last_update_time = time.time()

    def get_latest_data(self):
        with self.lock:
            return {
                'imu': self.imu_data,
                'rpm': self.rpm_data,
                'linear_velocity': self.linear_velocity,
                'linear_velocity_std': self.linear_velocity_std,
                'timestamp': self.last_update_time
            }


def initialize_serial(port, baud_rate, timeout):
    """Inicializa la conexión serial y retorna el objeto serial."""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=timeout
        )
        print(f"Conectado al puerto {port} a {baud_rate} baudios.")
        time.sleep(2)  # Espera para que Arduino se inicialice
        return ser
    except serial.SerialException as e:
        print(f"Error al conectar al puerto {port}: {e}")
        return None

def send_rpm_command(ser, rpm1, rpm2, rpm3, rpm4):
    """Envía una cadena con los RPM de las 4 llantas en el formato M1:rpm1;M2:rpm2;M3:rpm3;M4:rpm4\n."""
    if ser is None or not ser.is_open:
        print("Error: No hay conexión serial activa.")
        return

    # Formatear la cadena
    command = f"M1:{rpm1};M2:{rpm2};M3:{rpm3};M4:{rpm4}\n"
    try:
        # Enviar la cadena codificada
        ser.write(command.encode('utf-8'))
        print(f"Enviado: {command.strip()}")
    except serial.SerialException as e:
        print(f"Error al enviar el comando: {e}")
