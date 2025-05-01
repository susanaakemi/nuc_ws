#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class RockDetectorNode(Node):
    def __init__(self):
        super().__init__('rock_detector_node')
        # 1) Subscripción a la imagen raw IR
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        # 2) Publicador de la coordenada central
        self.center_pub = self.create_publisher(
            Point,
            'rock_center_px',
            10
        )
        self.bridge = CvBridge()
        # 3) Cargar el modelo YOLOv8 (ajusta 'path/to/your_model.pt')
        self.model = YOLO('/home/robotec/ros2_ws/src/camera_pkg/models/best.pt')

    def image_callback(self, msg: Image):
        # Convertir ROS Image a OpenCV
        try:
		cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		results = self.model(cv_img)
        except Exception as e:
            self.get_logger().error(f'Error CvBridge: {e}')
            return

        # Inferencia con YOLOv8
        results = self.model(cv_img)
        if len(results) == 0 or len(results[0].boxes) == 0:
            # No se detectó ninguna roca
            return

        # Tomamos la primera detección (podrías iterar si quieres todas)
        box = results[0].boxes.xyxy[0]  # [x_min, y_min, x_max, y_max]
        x_min, y_min, x_max, y_max = box.cpu().numpy()

        # Calcular centro de la caja
        cx = float(x_min + (x_max - x_min) / 2.0)
        cy = float(y_min + (y_max - y_min) / 2.0)

        # Publicar como geometry_msgs/Point
        pt_msg = Point()
        pt_msg.x = cx
        pt_msg.y = cy
        pt_msg.z = 0.0
        self.center_pub.publish(pt_msg)

        self.get_logger().info(f'Publicado centro: ({cx:.1f}, {cy:.1f})')

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
