#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class TestRpmPublisher(Node):
    def __init__(self):
        super().__init__('test_rpm_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, 'target_rpms', 10)
        self.timer_period = 0.1  # 100 ms = 10 Hz
        self.start_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time

        if elapsed_time > 3.0:
            self.get_logger().info('3 segundos cumplidos. Deteniendo nodo.')
            rclpy.shutdown()
            return

        if elapsed_time <= 1.0:
            rpm_values = [100.0, 100.0, 100.0, 100.0]  # avanzar
        elif elapsed_time <= 2.0:
            rpm_values = [0.0, 0.0, 0.0, 0.0]    # detenerse
        else:
            rpm_values = [-100.0, -100.0, -100.0, -100.0]  # retroceder

        # Publicar en el tópico
        msg = Float32MultiArray()
        msg.data = rpm_values
        self.publisher.publish(msg)

        # Imprimir en formato serial
        command = f"D1:{int(rpm_values[0])};D2:{int(rpm_values[1])};D3:{int(rpm_values[2])};D4:{int(rpm_values[3])}"
        self.get_logger().info(f"Enviado (simulado serial): {command}")

def main(args=None):
    rclpy.init(args=args)
    node = TestRpmPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
