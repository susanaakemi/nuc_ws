import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import serial
import time
import json

def mm(x):  # Convert meters to mm
    return int(x * 1000)

arm_forward_offset = 0.15
camera_height = 0.5
arm_base_height = 0.3

class ClickedPointSubscriber(Node):
    def __init__(self):
        super().__init__('clicked_point_subscriber')

        # Setup serial
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(2)
        self.get_logger().info("Serial connection established.")

        # Subscription
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.listener_callback,
            10
        )

        # Fixed positions
        self.drop_coords = { "x": mm(0.1), "y": mm(-0.1), "z": mm(0.02) } #Cambiar coordenadas
        self.home_coords = { "x": mm(0.0), "y": mm(0.0), "z": mm(0.1) } #Cambiar coordenadas

    def send_command(self, cmd):
        json_cmd = json.dumps(cmd)
        self.ser.write((json_cmd + '\n').encode('utf-8'))
        self.get_logger().info(f"Sent: {json_cmd}")
        time.sleep(1.5)

    def listener_callback(self, msg):
        x, y, z = mm(msg.point.x), mm(msg.point.y), mm(msg.point.z)
        self.get_logger().info(f'Recibido: x={x-arm_forward_offset}, y={y+(camera_height-arm_base_height)}, z={z}')

        # === Pick and Place Sequence ===

        # 1. Open gripper
        self.send_command({ "T": 1042, "id": 4, "angle": 3.14 })

        # 2. Move to stone
        self.send_command({ "T": 1041, "x": x, "y": y, "z": z, "t": 0.0 })

        # 3. Close gripper
        self.send_command({ "T": 1042, "id": 4, "angle": 0.0 })

        # 4. Move to drop-off
        self.send_command({ "T": 1041, **self.drop_coords, "t": 0.0 })

        # 5. Open gripper to drop
        self.send_command({ "T": 1042, "id": 4, "angle": 3.14 })

        # 6. Return to home
        self.send_command({ "T": 1041, **self.home_coords, "t": 0.0 })

        self.get_logger().info("Cycle complete.")

def main(args=None):
    rclpy.init(args=args)
    node = ClickedPointSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
