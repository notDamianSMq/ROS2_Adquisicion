#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import HistoryPolicy

import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

from project_interfaces.msg import Detections, Detection
from collections import Counter


class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')

        # --- Cargar YOLO ---
        self.get_logger().info("Cargando modelo YOLO...")
        self.model = YOLO("yolov8n.pt")
        self.get_logger().info("YOLO cargado correctamente.")

        self.bridge = CvBridge()

        # Suscripción cámara
        self.create_subscription(
            Image,
            '/my_camera/pylon_ros2_camera_node/image_raw',
            self.listener_callback,
            HistoryPolicy.KEEP_LAST
        )

        # Publicador detecciones
        self.publisher = self.create_publisher(
            Detections,
            '/image/predictions',
            10
        )

        self.get_logger().info("Nodo procesador YOLO iniciado.")

    def listener_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Inferencia YOLO
        results = self.model(frame, verbose=False)
        pred = results[0]

        detections_msg = Detections()
        detections_msg.num_detections = len(pred.boxes)

        # Contador de etiquetas
        class_counter = Counter()

        for box in pred.boxes:
            d = Detection()

            # bounding box
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
            d.x1 = float(x1)
            d.y1 = float(y1)
            d.x2 = float(x2)
            d.y2 = float(y2)

            # clase
            cls_id = int(box.cls[0])
            d.class_id = cls_id
            d.class_name = self.model.names[cls_id]
            class_counter[d.class_name] += 1

            # confianza
            d.confidence = float(box.conf[0])

            detections_msg.detections.append(d)

        # Publicar
        self.publisher.publish(detections_msg)

        # LOG DETALLADO
        if detections_msg.num_detections > 0:
            resumen = ", ".join([f"{cnt} {name}" for name, cnt in class_counter.items()])
        else:
            resumen = "sin etiquetas detectadas"

        self.get_logger().info(
            f"Publicado: {detections_msg.num_detections} detecciones → {resumen}"
        )


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ImageProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nodo finalizado.")
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
