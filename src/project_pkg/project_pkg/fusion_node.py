#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import matplotlib.pyplot as plt

from project_interfaces.msg import CleanedCloud

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.points_subscriber = self.create_subscription(
            CleanedCloud, "/clean/points", self.points_callback, 10
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

    def points_callback(self, msg: CleanedCloud):
        points = msg.points
        clean_precision = msg.clean_precision

        self.get_logger().info(
            f'Recibido PointCloud2: ancho={points.width}, alto={points.height}, campos={len(points.fields)}, Limpiado con precision {clean_precision}\n'
        )

        self.add_frame_avi(points)

    def add_frame_avi(self, msg: PointCloud2):
        # Convertir PointCloud2 a lista de puntos
        points = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        if points.size == 0:
            return

        # Normalizar y escalar para visualizar
        xs = np.array([t[0] for t in points])
        ys = np.array([t[1] for t in points])
        zs = np.array([t[2] for t in points])

        scale = 50.0  # metros visibles en X e Y (ajústalo según tu entorno)
        half_scale = scale / 2.0

        # Proyección simple: X-Y
        img = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)

        # Filtro: solo puntos dentro del rango visible
        mask = (np.abs(xs) <= half_scale) & (np.abs(ys) <= half_scale)
        xs = xs[mask]
        ys = ys[mask]
        zs = zs[mask]

        # Convertir de metros → píxeles (sin reescalar por frame)
        px_per_meter_x = self.frame_width / scale
        px_per_meter_y = self.frame_height / scale

        # Trasladar el origen (0,0) al centro de la imagen
        u = (self.frame_width / 2 + xs * px_per_meter_x).astype(int)
        v = (self.frame_height / 2 - ys * px_per_meter_y).astype(int)  # invertimos Y para que arriba sea +Y

        # Dibujar puntos válidos dentro de la imagen
        valid = (u >= 0) & (u < self.frame_width) & (v >= 0) & (v < self.frame_height)
        u, v, zs = u[valid], v[valid], zs[valid]

        # --- Asignar color según altura (Z) ---
        # Normalizar Z entre 0 y 1
        z_min, z_max = -3.0, 8.0  # ajusta a tu sensor
        z_norm = np.clip((zs - z_min) / (z_max - z_min), 0, 1)

        # Mapear a colormap (usa OpenCV)
        colors = (255 * plt.cm.jet(z_norm)[:, :3]).astype(np.uint8)  # usa 'jet' o 'viridis'

        # Dibujar los puntos con color
        for (x, y, c) in zip(u, v, colors):
            img[y, x] = c[::-1]  # convertir de RGB (matplotlib) a BGR (OpenCV)

        cv2.imshow("Nube de puntos XY (color por Z)", img)
        cv2.waitKey(1)
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