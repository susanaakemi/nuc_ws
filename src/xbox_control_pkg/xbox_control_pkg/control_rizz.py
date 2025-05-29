import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

def map_range(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def joy_callback(msg: Joy, logger):
    # Left joystick
    left_stick_x = msg.axes[0]

    # Map left_stick_x from [-1, 1] to [-90, 90]
    left_stick_x_mapped = map_range(left_stick_x, -1, 1, -90, 90)

    # Triggers
    lt = msg.axes[5] if len(msg.axes) > 5 else 0.0
    rt = msg.axes[4] if len(msg.axes) > 4 else 0.0

    # Map triggers from [1, -1] to [0, -50] for LT and [0, 50] for RT
    lt_mapped = map_range(lt, -1, 1, -50, 0)
    rt_mapped = map_range(rt, -1, 1, 50, 0)

    return left_stick_x_mapped, lt_mapped, rt_mapped

def main(args=None):
    rclpy.init(args=args)
    node = Node('xbox_controller_reader')

    node.create_subscription(
        Joy,
        '/joy',
        left_stick, lt, rt = lambda msg: joy_callback(msg, node.get_logger()),
        self.get_logger().info(
            f'Left Stick: x={left_stick:.2f} | '
            f'LT: {lt:.2f} | RT: {rt:.2f}'
        )
        10
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
