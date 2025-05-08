#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class RpmPublisher(Node):
    def __init__(self):
        super().__init__('rpmtrue_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, 'target_rpms', 10)
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9  # segundos

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time

        msg = Float32MultiArray()

        if elapsed <= 3.0:
            msg.data = [60.0, 60.0, 60.0, 60.0]  # avanzar
        elif elapsed <= 6.0:
            msg.data = [-60.0, -60.0, -60.0, -60.0]  # retroceder
        elif elapsed <= 9.0:
            msg.data = [0.0, 0.0, 0.0, 0.0]  # detener
        else:
            self.get_logger().info("Fin de la prueba. Nodo se detiene.")
            rclpy.shutdown()
            return

        self.publisher.publish(msg)
        self.get_logger().info(f"Publicado: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RpmPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

