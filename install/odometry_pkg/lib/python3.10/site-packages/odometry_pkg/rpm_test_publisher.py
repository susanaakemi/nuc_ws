#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class RpmSubscriber(Node):
    def __init__(self):
        super().__init__('rpm_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'target_rpms',
            self.listener_callback,
            10)
        self.subscription  # evita warning de variable no usada

    def listener_callback(self, msg):
        rpm_values = msg.data
        self.get_logger().info(f"Recibido del tópico target_rpms: {rpm_values}")

def main(args=None):
    rclpy.init(args=args)
    node = RpmSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

