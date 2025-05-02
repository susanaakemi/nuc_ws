#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer  # [A]
from rclpy.qos import QoSProfile, SensorDataQoS                    # [A]
from ultralytics import YOLO
import cv2

class RockDetectorNode(Node):
    def __init__(self):
        super().__init__('rock_detector_node')

        # [B] Parámetros configurables vía CLI/launch
        self.declare_parameter('model_path', '/home/robotec/ros2_ws/src/camera_pkg/models/best.pt')
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('frame_skip', 5)

        model_path = self.get_parameter('model_path').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.frame_skip = int(self.get_parameter('frame_skip').value)

        # Instancias
        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        # [C] QoS: imágenes de cámara suelen ser sensor data
        img_qos = SensorDataQoS()
        # Publicaciones propias pueden ser reliable
        pub_qos = QoSProfile(depth=10)

        # [D] Suscriptor con QoS
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile=img_qos
        )

        # Publicador de centro(s) detectado(s)
        self.center_pub = self.create_publisher(
            Point,
            'rock_center_px',
            qos_profile=pub_qos
        )

        # Contador para hacer frame skipping
        self._frame_count = 0

    def image_callback(self, msg: Image):
        # [E] Procesar solo 1 de cada N frames para ahorrar CPU
        self._frame_count = (self._frame_count + 1) % self.frame_skip
        if self._frame_count != 0:
            return

        # Convertir ROS -> OpenCV
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error CvBridge: {e}')
            return

        # [F] Inferencia única con umbral de confianza
        try:
            results = self.model(cv_img, conf=self.conf_thres)
        except Exception as e:
            self.get_logger().error(f'Error en inferencia YOLO: {e}')
            return

        if not results or not results[0].boxes:
            # Sin detecciones útiles
            return

        # [G] Iterar sobre todas las detecciones (no solo la primera)
        for box in results[0].boxes.xyxy.cpu().numpy():
            x_min, y_min, x_max, y_max = box
            cx = float((x_min + x_max) * 0.5)
            cy = float((y_min + y_max) * 0.5)

            pt_msg = Point()
            pt_msg.x = cx
            pt_msg.y = cy
            pt_msg.z = 0.0
            self.center_pub.publish(pt_msg)
            self.get_logger().info(f'Publicado centro roca: ({cx:.1f}, {cy:.1f})')

def main(args=None):
    rclpy.init(args=args)
    node = RockDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()