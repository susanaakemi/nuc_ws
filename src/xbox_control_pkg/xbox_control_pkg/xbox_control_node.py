import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import time
# import serial  

class XboxControllerReader(Node):
    def __init__(self):
        super().__init__('Xbox_controller_reader')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.rpm_publisher = self.create_publisher(Float32MultiArray, '/target_rpms', 10)
        # self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        self.get_logger().info("Serial communication disabled for testing")

    def define_rpms(self, delta, v2, v1):
        rpm_exterior = (v2 / (np.pi * 0.24)) * 60
        rpm_interior = (v1 / (np.pi * 0.24)) * 60
        if delta > 0:
            rpm_left = rpm_interior
            rpm_right = rpm_exterior
        elif delta < 0:
            rpm_left = rpm_exterior
            rpm_right = rpm_interior
        else:
            rpm_left = rpm_right = rpm_exterior
        self.send_rpm([rpm_left, -rpm_right, rpm_left, -rpm_right])

    def map_range(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def joy_callback(self, msg: Joy):
        left_stick_x = msg.axes[0]
        deadzone = 0.1
        left_stick_x = 0.0 if abs(left_stick_x) < deadzone else left_stick_x
        steering_angle_deg = self.map_range(left_stick_x, -1, 1, -90, 90)

        lt = msg.axes[5] if len(msg.axes) > 5 else 0.0
        rt = msg.axes[4] if len(msg.axes) > 4 else 0.0

        lt_mapped = self.map_range(lt, -1, 1, -50, 0)
        rt_mapped = self.map_range(rt, -1, 1, 50, 0)

        if rt_mapped != 0:
            lt_mapped = 0
        elif lt_mapped != 0:
            rt_mapped = 0

        outer_speed = rt_mapped + lt_mapped
        v2 = (np.pi * 0.24 * outer_speed) / 60

        delta = np.radians(steering_angle_deg)
        front_ackermann_angle, v1 = self.angulo_ackermann(delta, v2)

        self.get_logger().info(f"LT: {lt}, RT: {rt}")  # temporal
        self.get_logger().info(
            f'[FRONT] Input steering angle: {steering_angle_deg:.2f}° | '
            f'Ackermann angle: {np.degrees(front_ackermann_angle):.2f}° | '
            f'Outer speed: {outer_speed:.2f} | Inner speed: {v1:.2f}'
        )

    def angulo_ackermann(self, delta, v2):
        if abs(delta) < 1e-3:
            self.send_angles([0.0, 0.0, 0.0, 0.0])
            self.define_rpms(0.0, v2, v2)
            return 0.0, v2

        # Parámetros geométricos
        width = 0.89     # distancia entre ruedas delanteras
        length = 0.2815  # distancia entre ejes
       
        R_int = length / np.tan(abs(delta))  # Radio de giro de la rueda interior
       
        R_ext = R_int + width # Radio de giro de la rueda exterior

        delta_ext = np.arctan(length / R_ext)# Ángulo exterior en radianes

        v1 = (R_int / R_ext) * v2# Velocidad de la rueda interior

        # Signos según giro
        if delta < 0:
            # Giro hacia la izquierda: rueda interior es derecha
            angles = [-delta_ext, -delta, delta_ext, delta]
        else:
            # Giro hacia la derecha: rueda interior es izquierda
            angles = [delta, delta_ext, -delta, -delta_ext]

        self.send_angles(angles)
        self.define_rpms(delta, v2, v1)
        return delta_ext, v1

    def send_angles(self, angles):
        angles4 = f"D1:{angles[0]};D2:{angles[1]};D3:{angles[2]};D4:{angles[3]}"
        self.get_logger().info(f"(DEBUG) Would send angles: {angles4}")
        # Desactivado para pruebas sin serial
        # try:
        #     self.serial_port.write(angles4.encode('UTF-8'))
        #     self.get_logger().info(f"Sent over shared serial (4 angles): {angles4.strip()}")
        #     time.sleep(0.05)
        # except serial.SerialException as e:
        #     self.get_logger().error(f"Serial write failed: {e}")

    def send_rpm(self, rpms):
        msg = Float32MultiArray()
        msg.data = rpms
        self.rpm_publisher.publish(msg)
        self.get_logger().info(f"Published RPMs: {rpms}")

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

