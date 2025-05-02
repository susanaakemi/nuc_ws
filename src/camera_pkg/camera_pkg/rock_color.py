#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge
    
class ColorDetectorNode(Node):
    def _init_(self):
        super()._init_('color_detector_node')
        # Subscripción a las coordenadas de las rocas
        self.center_sub = self.create_subscription(
            Point,
            'rock_center_px',
            self.center_callback,
            10
        )
        # Publicador del color de la roca
        self.color_pub = self.create_publisher(
            String,
            'rock_color',
            10
        )
        self.bridge = CvBridge()
        # Iniciar la cámara
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara.")
            return

        # Rangos de colores en HSV
        self.rangos_colores = {
            "Rojo1": [(0, 70, 50), (10, 255, 255)],  # Rojo brillante
            "Rojo2": [(170, 70, 50), (180, 255, 255)],  # Rojo brillante (ajuste para el espectro completo)
            "Verde": [(35, 50, 50), (90, 255, 255)],  # Verde
            "Azul": [(90, 50, 50), (140, 255, 255)]   # Azul
        }

    def center_callback(self, msg: Point):
        # Leer un frame de la cámara
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("No se pudo leer el frame de la cámara.")
            return

        # Recortar la región de interés (ROI) usando las coordenadas (x, y)
        x, y = int(msg.x), int(msg.y)
        tamaño = 30  # tamaño de la región alrededor del punto central
        x1, y1 = max(x - tamaño, 0), max(y - tamaño, 0)
        x2, y2 = min(x + tamaño, frame.shape[1]), min(y + tamaño, frame.shape[0])

        roi = frame[y1:y2, x1:x2]

        # Convertir a HSV para procesar los colores
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Detección de color
        color_detectado = self.detectar_color(hsv)

        # Publicar el color detectado
        color_msg = String()
        color_msg.data = color_detectado
        self.color_pub.publish(color_msg)

        self.get_logger().info(f"Publicado color: {color_detectado}")

    def detectar_color(self, hsv):
        detecciones = {}
        for color, (bajo, alto) in self.rangos_colores.items():
            mask = cv2.inRange(hsv, np.array(bajo), np.array(alto))
            detecciones[color] = cv2.countNonZero(mask)

        # Detectar el color con más presencia en la ROI
        color_detectado = max(detecciones, key=detecciones.get)

        # Unificar "Rojo1" y "Rojo2" en "Rojo"
        if "Rojo" in color_detectado:
            return "Rojo"
        
        # Solo devolver colores si hay suficientes píxeles del color detectado
        return color_detectado if detecciones[color_detectado] > 300 else "Desconocido"

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()