#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from project_interfaces.srv import CleanCloud
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class CleanCLoudServerNode(Node):
    def __init__(self):
        super().__init__('clean_cloud_server')
        self.create_service(CleanCloud, 'clean_pointcloud', self.clean_callback, )
        self.get_logger().info('Servidor clean_cloud_server iniciado.')

    def clean_callback(self, request, response):
        # Convertir PointCloud2 a numpy array
        points = pc2.read_points(request.input, field_names=("x", "y", "z"), skip_nans=True)

        if points.size == 0:
            self.get_logger().warn("⚠️ La nube de entrada está vacía")
            response.output = request.input
            return response

        points = np.array(points.tolist(), dtype=float)
        precision = max(0, request.precision)  # seguridad: evita valores negativos

        # Redondear los puntos según la precisión solicitada
        rounded = np.round(points, precision)
        # Eliminar duplicados
        unique_points = np.unique(rounded, axis=0)

        self.get_logger().info(
            f"🧹 Nube reducida: {len(points)} → {len(unique_points)} puntos únicos (precisión={precision})")

        # Crear nueva nube PointCloud2
        header = request.input.header
        fields = []

        for i, name in enumerate(['x', 'y', 'z']):
            f = PointField()
            f.name = name
            f.offset = i * 4  # float32 ocupa 4 bytes
            f.datatype = PointField.FLOAT32
            f.count = 1
            fields.append(f)

        response.output = pc2.create_cloud(header, fields, unique_points.tolist())

        self.get_logger().info(
            f"Enviando respuesta")
        return response

def main(args=None):
    try:
        rclpy.init(args=args)
        node = CleanCLoudServerNode()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()