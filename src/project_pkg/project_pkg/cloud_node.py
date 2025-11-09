#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class CloudNode(Node):
    def __init__(self):
        super().__init__('cloud_points_node')
        self.subscriber = self.create_subscription(
            PointCloud2, "/ouster/points", self.callback, 10
        )
        self.publisher = self.create_publisher(PointCloud2, "/clean/points", 10)
        self. i = 0
        self.get_logger().info('Nodo cloud_points_node iniciado. Escuchando /ouster/points...')

    def callback(self, msg: PointCloud2):
        self.i += 1
        self.get_logger().info(
            f'Recibido PointCloud2: ancho={msg.width}, alto={msg.height}, campos={len(msg.fields)}\n'
            f'Mensajes totales {self.i}'
        )

        # Publicar los datos una vez han sido limpiados
        self.get_logger().info("Transmitiendo datos limpiados\n")
        self.publisher.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CloudNode()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()