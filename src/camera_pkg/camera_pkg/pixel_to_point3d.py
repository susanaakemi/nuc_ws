#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import QoSProfile, SensorDataQoS
import ros2_numpy as rnp
import numpy as np

class PixelToPoint3D(Node):
    def __init__(self):
        super().__init__('pixel_to_point3d')

        # QoS específico para datos de sensor (nube de puntos)
        pc_qos = SensorDataQoS()
        # QoS genérico para coordenadas pixeladas
        pix_qos = QoSProfile(depth=10)

        # Subscriptores sincronizados por timestamp con QoS
        pix_sub = Subscriber(self, Point, '/rock_center_px', qos_profile=pix_qos)
        pc_sub  = Subscriber(self, PointCloud2, '/camera/depth/points', qos_profile=pc_qos)
        ats     = ApproximateTimeSynchronizer(
            [pix_sub, pc_sub],
            queue_size=10,
            slop=0.05  # tolerancia de ±50 ms
        )
        ats.registerCallback(self.synced_callback)

        # Publicador del punto 3D en metros
        self.point_pub = self.create_publisher(
            PointStamped,
            '/clicked_point',
            10
        )

    def synced_callback(self, pix: Point, pc_msg: PointCloud2):
        # 1) Extraer y validar píxel
        u, v = int(pix.x), int(pix.y)

        # 2) Convertir directamente a XYZ array (H x W x 3)
        try:
            xyz = rnp.point_cloud2_to_xyz_array(pc_msg)
        except Exception as e:
            self.get_logger().error(f'Error al convertir PointCloud2: {e}')
            return

        h, w, _ = xyz.shape
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warn(f'Pixel fuera de rango: (u={u}, v={v})')
            return

        # 3) Extraer coordenadas 3D y validar
        x, y, z = xyz[v, u]
        if np.isnan(x) or np.isinf(x):
            self.get_logger().warn('Punto inválido (NaN o Inf)')
            return

        # 4) Publicar PointStamped
        pt = PointStamped()
        pt.header = pc_msg.header  # conserva frame_id
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        self.point_pub.publish(pt)

        self.get_logger().info(f'Publicado punto 3D: ({x:.2f}, {y:.2f}, {z:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = PixelToPoint3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()