import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class XboxControllerReader(Node):
    def __init__(self):
        super().__init__('xbox_controller_reader')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def map_range(self, value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def joy_callback(self, msg: Joy):
        # Left joystick
        left_stick_x = msg.axes[0]   # Left stick horizontal (left = -1, right = 1)
  
        # Map left_stick_x from [-1, 1] to [-90, 90]
        left_stick_x_mapped = self.map_range(left_stick_x, -1, 1, -90, 90)

        # Triggers
        lt = msg.axes[5] if len(msg.axes) > 5 else 0.0
        rt = msg.axes[4] if len(msg.axes) > 4 else 0.0

        # Map triggers from [1, -1] to [0, -50] for LT and [0, 50] for RT
        lt_mapped = self.map_range(lt, -1, 1, -50, 0)
        rt_mapped = self.map_range(rt, -1, 1, 50, 0)

        self.get_logger().info(
            f'Left Stick: x={left_stick_x_mapped:.2f} | '
            f'LT: {lt_mapped:.2f} | RT: {rt_mapped:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

