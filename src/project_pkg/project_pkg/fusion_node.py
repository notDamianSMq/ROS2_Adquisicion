#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.points_subscriber = self.create_subscription(
            PointCloud2, "/clean/points", self.points_callback, 10
        )
        self.get_logger().info('Nodo fusion_node iniciado. Escuchando /clean/points...')

        # Configuración del video
        self.frame_width = 640
        self.frame_height = 480
        self.video_writer = cv2.VideoWriter(
            'cloud_video.avi',
            cv2.VideoWriter_fourcc(*'XVID'),
            10,  # fps
            (self.frame_width, self.frame_height)
        )

    def points_callback(self, msg: PointCloud2):
        self.get_logger().info(
            f'Recibido PointCloud2: ancho={msg.width}, alto={msg.height}, campos={len(msg.fields)}\n'
        )

        # Convertir PointCloud2 a lista de puntos
        points = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        if points.size == 0:
            return

        # Proyección simple: X-Y
        img = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)

        # Normalizar y escalar para visualizar
        xs = np.array([t[0] for t in points])
        ys = np.array([t[1] for t in points])
        #zs = np.array([t[2] for t in points])

        xs = ((xs - xs.min()) / (xs.max() - xs.min()) * (self.frame_width - 1)).astype(int)
        ys = ((ys - ys.min()) / (ys.max() - ys.min()) * (self.frame_height - 1)).astype(int)
        #zs = ((zs - zs.min()) / (zs.max() - zs.min()) * (self.frame_))

        # Dibujar puntos
        for x, y in zip(xs, ys):
            img[self.frame_height - 1 - y, x] = (0, 255, 0)  # verde

        self.video_writer.write(img)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = FusionNode()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()