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

        # Timer cada 0.2 segundos 
        self.timer = self.create_timer(0.2, self.try_publish_point)

        self.latest_pixel = None
        self.latest_pc = None

    def pixel_callback(self, msg: Point):
        self.latest_pixel = (int(msg.x), int(msg.y))
        self.get_logger().info(f"Recibido píxel: {self.latest_pixel}")

    def pc_callback(self, msg: PointCloud2):
        self.latest_pc = msg
        self.get_logger().info(f"Recibida nube de puntos: {msg.width}x{msg.height}")

    def try_publish_point(self):
        if self.latest_pixel is None or self.latest_pc is None:
            return

        u, v = self.latest_pixel
        width = self.latest_pc.width
        height = self.latest_pc.height

        if u < 0 or u >= width or v < 0 or v >= height:
            self.get_logger().warn(f'Pixel fuera de rango: (u={u}, v={v})')
            return

        if self.latest_pc.height <= 1:
            self.get_logger().warn('Point cloud is unorganized, cannot index by (u, v)')
            return

        try:
            pc_array = pc2.read_points_numpy(
                self.latest_pc,
                field_names=['x', 'y', 'z'],
                skip_nans=False
            )
            self.get_logger().info(f"pc_array shape: {pc_array.shape}")

            expected_size = width * height
            if pc_array.shape[0] != expected_size:
                self.get_logger().warn(
                    f'Unexpected pc_array size: {pc_array.shape[0]}, expected: {expected_size}'
                )
                return

            index = v * width + u
            if index >= len(pc_array):
                self.get_logger().warn(
                    f'Index {index} out of bounds for pc_array of size {len(pc_array)}'
                )
                return

            x, y, z = pc_array[index]

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

            self.latest_pixel = None

        except Exception as e:
            self.get_logger().error(f"Error al obtener punto 3D: {e}")
            return


def main(args=None):
    rclpy.init(args=args)
    node = PixelToPoint3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


