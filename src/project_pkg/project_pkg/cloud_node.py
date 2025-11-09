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
        self.get_logger().info('Nodo cloud_points_node iniciado. Escuchando /ouster/points...')

    def callback(self, msg: PointCloud2):
        self.get_logger().info(
            f'Recibido PointCloud2: ancho={msg.width}, alto={msg.height}, campos={len(msg.fields)}'
        )

def main(args=None):
    try:
        rclpy.init(args=args)
        node = CloudNode()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()