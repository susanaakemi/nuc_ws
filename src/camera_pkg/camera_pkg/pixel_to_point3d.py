import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import ros2_numpy as rnp
import numpy as np

class PixelToPoint3D(Node):
    def __init__(self):
        super().__init__('pixel_to_point3d')

        pc_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        pix_qos = QoSProfile(depth=10)

        pix_sub = Subscriber(self, PointStamped, '/rock_center_px', qos_profile=pix_qos)
        pc_sub = Subscriber(self, PointCloud2, '/camera/depth/points', qos_profile=pc_qos)
        ats = ApproximateTimeSynchronizer([pix_sub, pc_sub], queue_size=10, slop=0.05)
        ats.registerCallback(self.synced_callback)

        self.point_pub = self.create_publisher(PointStamped, '/clicked_point', 10)

    def synced_callback(self, pix: PointStamped, pc_msg: PointCloud2):
        u, v = int(pix.point.x), int(pix.point.y)

        try:
            points = rnp.numpify(pc_msg)

            if isinstance(points, dict):
                self.get_logger().info(f'points keys: {list(points.keys())}')
                array_xyz = points['xyz']
                is_dict = True
            else:
                self.get_logger().info(f'points dtype: {points.dtype}')
                self.get_logger().info(f'points shape: {points.shape}')
                self.get_logger().info(f'points first element: {points[0]}')
                is_dict = False

        except Exception as e:
            self.get_logger().error(f'Error al convertir PointCloud2: {e}')
            return

        if not (0 <= v < pc_msg.height and 0 <= u < pc_msg.width):
            self.get_logger().warn(f'Pixel fuera de rango: (u={u}, v={v})')
            return

        index = v * pc_msg.width + u

        try:
            if is_dict:
                x = array_xyz[index][0]
                y = array_xyz[index][1]
                z = array_xyz[index][2]
            else:
                x = points['x'][index]
                y = points['y'][index]
                z = points['z'][index]
        except Exception as e:
            self.get_logger().error(f'Error al acceder al punto 3D: {e}')
            return

        if np.isnan(x) or np.isinf(x) or np.isnan(y) or np.isinf(y) or np.isnan(z) or np.isinf(z):
            self.get_logger().warn('Punto inválido (NaN o Inf)')
            return

        pt = PointStamped()
        pt.header = pc_msg.header
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = float(z)
        self.point_pub.publish(pt)

        self.get_logger().info(f'Publicado punto 3D: ({x:.2f}, {y:.2f}, {z:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = PixelToPoint3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()