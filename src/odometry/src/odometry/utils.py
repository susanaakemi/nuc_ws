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
from odometry.msg import WheelRPMs
from odometry.msg import DirectionAngles

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

        # Parameters for velocity calculations
        self.wheel_radius = 0.05  # meters, adjust as needed
        self.ema_alpha = 0.1  # EMA smoothing factor
        self.prev_ema_std_v = 0.0  # Initialize previous EMA stddev

    def calculate_velocity_metrics(self, rpms, rpm_stddevs):
        """Calculate linear velocity and EMA-filtered stddev from RPMs and their stddevs."""
        # Linear velocity: RPM to angular velocity (rad/s) to linear velocity (m/s)
        linear_velocities = [
            (rpm * 2 * math.pi / 60) * self.wheel_radius for rpm in rpms
        ]
        self.linear_velocity = sum(linear_velocities) / len(linear_velocities)

        # Linear velocity stddev: Convert RPM stddev to linear velocity stddev
        linear_vel_stddevs = [
            (std * 2 * math.pi / 60) * self.wheel_radius for std in rpm_stddevs
        ]
        # Combine stddevs (assuming independence)
        variance_sum = sum(std ** 2 for std in linear_vel_stddevs) / len(linear_vel_stddevs)
        system_std_v = math.sqrt(variance_sum)
        # Apply EMA filtering
        ema_std_v = self.ema_alpha * system_std_v + (1 - self.ema_alpha) * self.prev_ema_std_v
        self.prev_ema_std_v = ema_std_v  # Update previous EMA

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

        # Check if data is available
        if self.rpms['stamp'] is None or self.imu_data['stamp'] is None:
            self.get_logger().warn("RPM or IMU data not available yet")
            return

        # Call specific data fields
        rpms = self.rpms['data']  # Four motor RPMs
        rpm_stddevs = self.stddev['data'][9:13]  # Stddevs of RPMs
        timestamp = self.imu_data['stamp'].sec + self.imu_data['stamp'].nanosec / 1e9

        # Calculate linear velocity and stddev
        linear_velocity, linear_velocity_stddev = self.calculate_velocity_metrics(rpms, rpm_stddevs)

        # Log the results
        self.get_logger().info(
            f"Velocity Data: Time={timestamp:.3f}, "
            f"Linear Vel={linear_velocity:.2f} m/s, "
            f"Linear Vel Stddev={linear_velocity_stddev:.2f} m/s"
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

class RPM_sender(Node):
    def __init__(self):
        super().__init__('rpm_publisher')
        self.publisher_ = self.create_publisher(WheelRPMs, 'wheel_rpm_cmd', 10)
        
    def send_rpm(self, rpm1, rpm2, rpm3, rpm4):
        msg = WheelRPMs()
        msg.rpm1 = rpm1
        msg.rpm2 = rpm2
        msg.rpm3 = rpm3
        msg.rpm4 = rpm4
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published RPMs: {rpm1}, {rpm2}, {rpm3}, {rpm4}")
        
class Angle_sender(Node):
    def __init__(self):
        super().__init__("angle_publisher")
        self.publisher_=self.create_publisher(DirectionAngles, "angle_rpm_cmd",10)
        
    def send_angles(self,a_1,a_2,a_3,a_4):
        msg = DirectionAngles()
        msg.angle_1 = a_1
        msg.angle_2 = a_2
        msg.angle_3 = a_3
        msg.angle_4 = a_4
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Angles: {a_1}, {a_2}, {a_3}, {a_4}")
        
