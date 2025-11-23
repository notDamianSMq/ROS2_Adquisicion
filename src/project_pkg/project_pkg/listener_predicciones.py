#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from image_interfaces.msg import Detections, Detection


class PredictionsListener(Node):
    def __init__(self):
        super().__init__('predictions_listener')

        self.subscription = self.create_subscription(
            Detections,
            '/image/predictions',
            self.cb,
            10
        )

        self.get_logger().info("Nodo listener de predicciones iniciado.")

    def cb(self, msg: Detections):
        self.get_logger().info(f"Recibido: {msg.num_detections} detecciones:")

        for d in msg.detections:
            self.get_logger().info(
                f" - {d.class_name} (conf={d.confidence:.2f}) "
                f"[{d.x1:.1f}, {d.y1:.1f}, {d.x2:.1f}, {d.y2:.1f}]"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PredictionsListener()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
