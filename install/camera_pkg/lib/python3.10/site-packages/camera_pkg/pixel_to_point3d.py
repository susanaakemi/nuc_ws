import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
import ros2_numpy as rnp
import numpy as np

class PixelToPoint3D(Node):
    def __init__(self):
        super().__init__('pixel_to_point3d')

        # Suscripción a coordenadas de píxel (en realidad, publicadas como geometry_msgs/Point)
        self.pixel_sub = self.create_subscription(
            Point,
            '/rock_center_px',  # Este es el tópico publicado por RockDetectorNode
            self.pixel_callback,
            10)

        # Suscripción a la nube de puntos de la cámara
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pc_callback,
            10)

        # Publicador del punto 3D en metros
        self.point_pub = self.create_publisher(
            PointStamped,
            '/clicked_point',
            10)

        self.latest_pixel = None  # (u, v)
        self.latest_pc = None     # PointCloud2

    def pixel_callback(self, msg: Point):
        self.latest_pixel = (int(msg.x), int(msg.y))
        self.try_publish_point()

    def pc_callback(self, msg):
        self.latest_pc = msg
        self.try_publish_point()

    def try_publish_point(self):
        if self.latest_pixel is None or self.latest_pc is None:
            return

        u, v = self.latest_pixel
        pc = rnp.numpify(self.latest_pc)
        width = self.latest_pc.width
        height = self.latest_pc.height

        if u < 0 or u >= width or v < 0 or v >= height:
            self.get_logger().warn(f'Pixel fuera de rango: (u={u}, v={v})')
            return

        index = v * width + u
        point = pc.reshape(-1)[index]
        x = point['x']
        y = point['y']
        z = point['z']

        if any(np.isnan([x, y, z])) or any(np.isinf([x, y, z])):
            self.get_logger().warn('Punto inválido (NaN o Inf)')
            return

        # Publicar como coordenada 3D en metros
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.latest_pc.header.frame_id
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z

        self.point_pub.publish(point_msg)
        self.get_logger().info(f'Publicado clicked_point (m): ({x:.2f}, {y:.2f}, {z:.2f})')

        self.latest_pixel = None  # Espera una nueva coordenada

def main(args=None):
    rclpy.init(args=args)
    node = PixelToPoint3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()