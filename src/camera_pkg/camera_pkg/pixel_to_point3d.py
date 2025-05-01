import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np


class PixelToPoint3D(Node):
    def __init__(self):
        super().__init__('pixel_to_point3d')

        self.pixel_sub = self.create_subscription(
            Point,
            '/rock_center_px',
            self.pixel_callback,
            10
        )

        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pc_callback,
            10
        )

        self.point_pub = self.create_publisher(
            PointStamped,
            '/clicked_point',
            10
        )

        self.latest_pixel = None
        self.latest_pc = None

    def pixel_callback(self, msg: Point):
        self.latest_pixel = (int(msg.x), int(msg.y))
        self.get_logger().info(f"Recibido píxel: {self.latest_pixel}")
        self.try_publish_point()

    def pc_callback(self, msg: PointCloud2):
        self.latest_pc = msg
        self.get_logger().info(f"Recibida nube de puntos: {msg.width}x{msg.height}")
        self.try_publish_point()

    def try_publish_point(self):
        if self.latest_pixel is None or self.latest_pc is None:
            return

        u, v = self.latest_pixel
        width = self.latest_pc.width
        height = self.latest_pc.height

        if u < 0 or u >= width or v < 0 or v >= height:
            self.get_logger().warn(f'Pixel fuera de rango: (u={u}, v={v})')
            return

        try:
            # Read only the single point at pixel (u, v)
            pts = pc2.read_points_numpy(
                self.latest_pc,
                field_names=['x', 'y', 'z'],
                uvs=[(u, v)],
                skip_nans=False,
                reshape_organized_cloud=False
            )
            x, y, z = pts[0]
        except Exception as e:
            self.get_logger().error(f"Error al obtener punto 3D: {e}")
            return

        if any(np.isnan([x, y, z])) or any(np.isinf([x, y, z])):
            self.get_logger().warn('Punto inválido (NaN o Inf)')
            return

        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.latest_pc.header.frame_id
        point_msg.point.x = float(x)
        point_msg.point.y = float(y)
        point_msg.point.z = float(z)

        self.point_pub.publish(point_msg)
        self.get_logger().info(
            f'Publicado clicked_point (m): ({x:.2f}, {y:.2f}, {z:.2f})'
        )

        # reset for next pixel
        self.latest_pixel = None


def main(args=None):
    rclpy.init(args=args)
    node = PixelToPoint3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

