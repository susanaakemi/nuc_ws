import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class XboxControlNode(Node):
    def __init__(self):
        super().__init__('xbox_control_node')

        self.rpm_pub = self.create_publisher(Float32MultiArray, '/target_rpms', 10)
        self.angle_pub = self.create_publisher(Float32MultiArray, '/target_angles', 10)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        speed_input = msg.axes[1]  # Adelante/atrás (stick izq vertical)
        steer_input = msg.axes[3]  # Giro (stick der horizontal)

        # Mapear a RPMs
        base_rpm = speed_input * 1000  # escala [-1000, 1000] rpm

        # Mapear a ángulos
        max_angle = 0.5  # máximo en radianes
        steer_angle = steer_input * max_angle

        rpm_msg = Float32MultiArray()
        rpm_msg.data = [base_rpm, base_rpm, base_rpm, base_rpm]

        angle_msg = Float32MultiArray()
        angle_msg.data = [steer_angle, steer_angle, steer_angle * 0.5, steer_angle * 0.5]

        self.rpm_pub.publish(rpm_msg)
        self.angle_pub.publish(angle_msg)

        self.get_logger().info(f"RPMs enviados: {rpm_msg.data}")
        self.get_logger().info(f"Ángulos enviados: {angle_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = XboxControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
