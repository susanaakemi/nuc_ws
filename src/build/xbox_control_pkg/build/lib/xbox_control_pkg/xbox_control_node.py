import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class DoubleAckermannControlNode(Node):
    def __init__(self):
        super().__init__('double_ackermann_control_node')

        # Publicadores
        self.rpm_pub = self.create_publisher(Float32MultiArray, '/target_rpms', 10)
        self.angle_pub = self.create_publisher(Float32MultiArray, '/target_angles', 10)

        # Parámetros físicos del robot
        self.L = 0.89  # distancia entre ejes (m)
        self.wheel_diameter = 0.24  # diámetro de rueda (m)
        self.wheel_circumference = np.pi * self.wheel_diameter
        self.max_speed = 2.0  # m/s
        self.max_steer_angle = 0.52  # rad (~30°)

        # Variables de control
        self.latest_steer_input = 0.0
        self.forward_pressed = False
        self.backward_pressed = False

        # Suscripción al joystick
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Timer para publicar a 50 Hz
        self.timer = self.create_timer(0.02, self.publish_commands)

    def joy_callback(self, msg):
        self.latest_steer_input = msg.axes[0]  # Stick izquierdo horizontal
        self.forward_pressed = msg.buttons[5] == 1  # Botón RB
        self.backward_pressed = msg.buttons[4] == 1  # Botón LB

    def compute_ackermann(self, delta, desired_speed):
        if np.abs(delta) < 1e-5:
            return desired_speed, desired_speed

        R = self.L / np.tan(np.abs(delta))
        R_in = R - (self.L / 2)
        R_out = R + (self.L / 2)

        v_interior = desired_speed * (R_in / R_out)
        return v_interior, desired_speed

    def publish_commands(self):
        # Determinar velocidad según los botones
        if self.forward_pressed and not self.backward_pressed:
            speed = self.max_speed
        elif self.backward_pressed and not self.forward_pressed:
            speed = -self.max_speed
        else:
            speed = 0.0

        steer = self.latest_steer_input * self.max_steer_angle

        # Calcular velocidades para ruedas interiores y exteriores
        v_interior, v_exterior = self.compute_ackermann(steer, np.abs(speed))

        # Convertir a RPM
        rpm_exterior = (v_exterior / self.wheel_circumference) * 60
        rpm_interior = (v_interior / self.wheel_circumference) * 60

        if steer > 0:  # giro a la izquierda
            rpm_left = rpm_interior
            rpm_right = rpm_exterior
        elif steer < 0:  # giro a la derecha
            rpm_left = rpm_exterior
            rpm_right = rpm_interior
        else:  # recto
            rpm_left = rpm_right = rpm_exterior

        rpm_left *= np.sign(speed)
        rpm_right *= np.sign(speed)

        # Calcular ángulos por rueda (delanteros y traseros reflejados)
        front_left_angle = steer
        front_right_angle = steer
        rear_left_angle = -steer  # traseros reflejan en doble Ackermann
        rear_right_angle = -steer

        # Preparar mensajes
        rpm_values = [rpm_left, rpm_right, rpm_left, rpm_right]
        angle_values = [front_left_angle, front_right_angle, rear_left_angle, rear_right_angle]

        rpm_msg = Float32MultiArray()
        rpm_msg.data = rpm_values
        self.rpm_pub.publish(rpm_msg)

        angle_msg = Float32MultiArray()
        angle_msg.data = angle_values
        self.angle_pub.publish(angle_msg)

        self.get_logger().debug(f"Published RPMs: {rpm_values}")
        self.get_logger().debug(f"Published Angles: {angle_values}")

def main(args=None):
    rclpy.init(args=args)
    node = DoubleAckermannControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
